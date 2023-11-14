"""--------------------------------------------------------------------------
    This file is meant to parse the detected data.
   --------------------------------------------------------------------------"""

''' IMPORTS '''
# imports to download and parse an OSM map
import requests
import os
import xml.etree.ElementTree as ET
import xmltodict

# API files
import ldm.utils as utils

# numpy for math
import numpy as np

# pymap3d to convert from geodetic to enu and viceversa
import pymap3d as pm

# timedelta to add time to timestamps
from datetime import timedelta

''' FUNCTIONS '''
# Parse OpenStreetMap XML file
def osm(tx, radius, centre, timestamp):
    # Build path name
    dirname  = os.path.dirname(__file__)
    filename = os.path.join(dirname, 'osm.nosync/xml/' +
                            "-".join(str(coord) for coord in centre) + '.xml')

    # If the OSM file doesn't already exist download it
    if not os.path.exists(filename):
        f      = open(filename, "xb")
        around = "around:" + str(radius) + "," + ",".join(str(e) for e in centre)

        overpass_url = "http://localhost/api/interpreter" # local OSM server from docker overpass api
        overpass_query = """
        [out:xml];
        (way[highway]("""+around+""");
         >;
        );
        out;
        """
        while True:
            try:
                response = requests.get(overpass_url, params={'data': overpass_query})
            except requests.exceptions.RequestExceptions as e:
                pass
            break

        root = ET.fromstring(response.text)
        tree = ET.ElementTree(root)
        tree.write(f)
        f.close()

    # Open the XML file and convert it to a dict of lists of dicts
    osm_file = open(filename, 'r')
    osm_xml  = osm_file.read()
    osm_dict = xmltodict.parse(osm_xml, attr_prefix='', dict_constructor=dict)
    osm_file.close()

    # Extract data from the OSM file
    nodes     = osm_dict['osm']['node'] if 'node' in osm_dict['osm'] else []
    ways      = osm_dict['osm']['way'] if 'way' in osm_dict['osm'] else []
    relations = osm_dict['osm']['relation'] if 'relation' in osm_dict['osm'] else []

    # Required data for Geodetic to ECEF conversion
    wgs84_a = 6378137.0
    wgs84_b = 6356752.314245
    b2_a2   = (wgs84_b/wgs84_a)**2
    e2      = 1 - b2_a2

    # Add nodes
    if nodes:
        tx.run('''
            UNWIND $nodes as node

            CALL {
                WITH node

                WITH node, $wgs84_a/sqrt(1 - $e2*sin(radians(toFloat(node.lat)))^2) as N,
                        toFloat(node.lat) as lat, toFloat(node.lon) as lon
                WHERE node.tag IS NULL

                MERGE (n:OSMNode:L1 {ID:toInteger(node.id)})
                SET n.Lat = lat, n.Lon = lon, n.timestamp = $timestamp,
                    n.ecef_x = N*cos(radians(lat))*cos(radians(lon)),
                    n.ecef_y = N*cos(radians(lat))*sin(radians(lon)),
                    n.ecef_z = $b2_a2*N*sin(radians(lat))
                
                WITH n
                SET n.enu_x = (n.ecef_x - $lcpo[3])*$ecef_to_enu[0] + (n.ecef_y - $lcpo[4])*$ecef_to_enu[1],
                    n.enu_y = (n.ecef_x - $lcpo[3])*$ecef_to_enu[3] + (n.ecef_y - $lcpo[4])*$ecef_to_enu[4] + (n.ecef_z - $lcpo[5])*$ecef_to_enu[5]

                WITH n
                SET n.pos = point({x:n.enu_x, y:n.enu_y})

                RETURN true as no_tag

                UNION

                WITH node

                WITH node, $wgs84_a/sqrt(1 - $e2*sin(radians(toFloat(node.lat)))^2) as N,
                        toFloat(node.lat) as lat, toFloat(node.lon) as lon
                WHERE node.tag IS NOT NULL

                MERGE (n:OSMNode:L2 {ID:toInteger(node.id)})
                SET n.Lat = lat, n.Lon = lon, n.timestamp = $timestamp,
                    n.ecef_x = N*cos(radians(lat))*cos(radians(lon)),
                    n.ecef_y = N*cos(radians(lat))*sin(radians(lon)),
                    n.ecef_z = $b2_a2*N*sin(radians(lat))

                WITH n, node
                SET n.enu_x = (n.ecef_x - $lcpo[3])*$ecef_to_enu[0] + (n.ecef_y - $lcpo[4])*$ecef_to_enu[1],
                    n.enu_y = (n.ecef_x - $lcpo[3])*$ecef_to_enu[3] + (n.ecef_y - $lcpo[4])*$ecef_to_enu[4] + (n.ecef_z - $lcpo[5])*$ecef_to_enu[5]
                
                WITH n, node
                SET n.pos = point({x:n.enu_x, y:n.enu_y})

                WITH n, node.tag as tags

                UNWIND tags as tag

                MERGE (t:OSMTag {key:tag.k, value:tag.v})
                SET t.timestamp = $timestamp
                MERGE (n)-[:HAS_TAG]->(t)

                RETURN false as no_tag
            }
            RETURN no_tag
            ''', nodes=nodes, timestamp=timestamp, wgs84_a=wgs84_a, b2_a2=b2_a2,
            e2=e2, lcpo=utils.lcpo, ecef_to_enu=utils.ecef_to_enu)

    # Add ways
    if ways:
        tx.run('''
            UNWIND $ways as way

            WITH toInteger(way.id) as id, way.nd as refs, way.tag as tags

            MERGE (w:OSMWay:L1 {ID:id})
            SET w.timestamp = $timestamp

            WITH w, refs, tags

            CALL {
                WITH w, tags

                UNWIND tags as tag

                MERGE (t:OSMTag {key:tag.k, value:tag.v})
                SET t.timestamp = $timestamp
                MERGE (w)-[:HAS_TAG]->(t)

                RETURN true as parsed_tags
            }

            WITH w, refs

            OPTIONAL MATCH (w)-[:HAS_TAG]-(t1:OSMTag {key:'oneway'})
            OPTIONAL MATCH (w)-[:HAS_TAG]-(t2:OSMTag {key:'junction', value:'roundabout'})
            OPTIONAL MATCH (w)-[:HAS_TAG]-(t3:OSMTag {key:'highway', value:'motorway'})

            CALL {
                WITH w, refs, t1, t2, t3
                
                WITH w, refs, t1
                WHERE t1 IS NULL 
                      OR 
                      t1.value = "no"

                FOREACH (i IN range(1, size(refs) - 1) |
                    MERGE (n1:OSMNode {ID:toInteger(refs[i - 1].ref)})
                    MERGE (n2:OSMNode {ID:toInteger(refs[i].ref)})
                    MERGE (n1)-[r1:NEXT_NODE]->(n2)
                    SET r1.distance = point.distance(n1.pos, n2.pos)
                    MERGE (n1)<-[r2:NEXT_NODE]-(n2)
                    SET r2.distance = point.distance(n1.pos, n2.pos)
                    MERGE (w)-[r3:HAS_NODE]->(n1)
                    SET r3.nd = i
                    MERGE (w)-[r4:HAS_NODE]->(n2)
                    SET r4.nd = i + 1)

                RETURN false as oneway

                UNION

                WITH w, refs, t1, t2, t3

                WITH w, refs, t1, t2, t3
                WHERE t1.value = 'yes'
                      OR 
                      t2 IS NOT NULL
                      OR 
                      t3 IS NOT NULL

                FOREACH (i IN range(1, size(refs) - 1) |
                    MERGE (n1:OSMNode {ID:toInteger(refs[i - 1].ref)})
                    MERGE (n2:OSMNode {ID:toInteger(refs[i].ref)})
                    MERGE (n1)-[r1:NEXT_NODE]->(n2)
                    SET r1.distance = point.distance(n1.pos, n2.pos)
                    MERGE (w)-[r2:HAS_NODE]->(n1)
                    SET r2.nd = i
                    MERGE (w)-[r3:HAS_NODE]->(n2)
                    SET r3.nd = i + 1)

                RETURN true as oneway

                UNION

                WITH w, refs, t1, t2, t3

                WITH w, refs, t1
                WHERE t1.value = -1

                FOREACH (i IN range(1, size(refs) - 1) |
                    MERGE (n1:OSMNode {ID:toInteger(refs[i - 1].ref)})
                    MERGE (n2:OSMNode {ID:toInteger(refs[i].ref)})
                    MERGE (n1)<-[r1:NEXT_NODE]-(n2)
                    SET r1.distance = point.distance(n1.pos, n2.pos)
                    MERGE (w)-[r2:HAS_NODE]->(n1)
                    SET r2.nd = size(refs) + 1 - i
                    MERGE (w)-[r3:HAS_NODE]->(n2)
                    SET r3.nd = size(refs) - i)

                RETURN true as oneway
            }

            RETURN true as parsed_ways
            ''', ways=ways, timestamp=timestamp)

    # Add relations
    if relations:
        tx.run('''
            UNWIND $relations as relation
            WITH toInteger(relation.id) as id, relation.member as members,
            relation.tag as tags
            MERGE (r:OSMRelation:L1 {ID:id})
            SET r.timestamp = $timestamp
            WITH r, members, tags
            CALL{
                WITH r, members
                UNWIND members as member
                WITH r, member
                WHERE member.type = 'node'
                MATCH (n:OSMNode {ID:toInteger(member.ref)})
                MERGE (r)-[:IS_RELATED {role:member.role}]->(n)
            }
            CALL{
                WITH r, members
                UNWIND members as member
                WITH r, member
                WHERE member.type = 'way'
                MATCH (w:OSMWay {ID:toInteger(member.ref)})
                MERGE (r)-[:IS_RELATED {role:member.role}]->(w)
            }
            CALL{
                WITH r, tags
                UNWIND tags as tag
                MERGE (t:OSMTag {key:tag.k, value:tag.v})
                MERGE (r)-[:HAS_TAG]->(t)
            }
            ''', relations=relations, timestamp=timestamp)

    # Refactor graph by deleting old OSM data
    tx.run('''
           OPTIONAL MATCH (n:OSMNode)
           WHERE n.timestamp < $timestamp

           OPTIONAL MATCH (w:OSMWay)
           WHERE w.timestamp < $timestamp

           OPTIONAL MATCH (r:OSMRelation)
           WHERE r.timestamp < $timestamp

           OPTIONAL MATCH (t:OSMTag)
           WHERE t.timestamp < $timestamp

           DETACH DELETE n, w, r, t
           ''', timestamp=timestamp)

# Add ego-vehicle data
def ego_vehicle(tx, data, timestamp):
    # Get properties from data
    prop = data.to_dict()

    # By default update the OSM data
    update_osm = True

    # Check that lcpo has been initialised
    if None not in utils.lcpo:
        # Convert from ECEF to ENU
        prop['enu_x'], prop['enu_y'], prop['enu_z'] = pm.ecef2enu(prop['ecef_x'], prop['ecef_y'], prop['ecef_z'], *utils.lcpo[0:3])

        # If the ego is more less 100 m from the lcpo don't update the map
        if np.sqrt(prop['enu_x']**2 + prop['enu_y']**2) < 100:
            update_osm = False

    # Update the OSM data
    if update_osm:
        # Update the local coordinate point origin
        utils.set_lcpo(prop['Lat'], prop['Lon'], 0,
                       prop['ecef_x'], prop['ecef_y'], prop['ecef_z'])

        # Convert from ECEF to ENU
        prop['enu_x'], prop['enu_y'], prop['enu_z'] = pm.ecef2enu(prop['ecef_x'], prop['ecef_y'], prop['ecef_z'], *utils.lcpo[0:3])

        # Update the OSM data
        osm(tx, 200, [prop['Lat'], prop['Lon']], timestamp)

    # Angle trickery thanks to me and:
    # - https://stackoverflow.com/questions/1311049/how-to-map-atan2-to-degrees-0-360
    # - https://stackoverflow.com/questions/1878907/how-can-i-find-the-difference-between-two-angles

    # Add ego data
    res = tx.run('''
           MERGE (a:AutonomousVehicle:L4 {ID:11112000})
           SET a += $prop, a.timestamp = $timestamp, a.pos = point({x:$prop.enu_x, y:$prop.enu_y})

           WITH a

           OPTIONAL MATCH (n2:OSMNode)-[n2_a:WAS_ON]-(a)-[a_n1:IS_ON]-(n1:OSMNode)
           OPTIONAL MATCH (a)-[a_w:IS_ON]-(w:OSMWay)

           WITH a, n1, n2, w, n2_a, a_n1, a_w

           CALL {
                WITH a, n1, n2, w, n2_a, a_n1, a_w

                WITH a, n1 as is_on, n2 as was_on, w as is_on_w, n2_a, a_n1, a_w
                WHERE (is_on IS NULL
                       AND
                       was_on IS NULL
                       AND
                       is_on_w IS NULL)
                      OR
                      abs(sin(radians(abs((((360 + degrees(atan2(is_on.enu_x - was_on.enu_x, is_on.enu_y - was_on.enu_y))) % 360 - (360 + degrees(atan2(a.enu_x - was_on.enu_x, a.enu_y - was_on.enu_y))) % 360) + 540) % 360 - 180)))*point.distance(a.pos, was_on.pos)) > 8
                
                DELETE n2_a, a_n1, a_w

                WITH a

                CALL {
                    WITH a

                    MATCH (n1:OSMNode)-[n1_r1:NEXT_NODE]->(n2:OSMNode), (n1)-[n1_r2:HAS_NODE]-(w:OSMWay)-[n2_r1:HAS_NODE]-(n2), (w:OSMWay)-[:HAS_TAG]-(t:OSMTag {key:'highway'})
                    WHERE (n1_r2.nd = n2_r1.nd + 1 OR n1_r2.nd = n2_r1.nd - 1)
                          AND NOT t.value IN $paths
                          AND
                          (abs((((360 + degrees(atan2(n1.enu_x - a.enu_x, n1.enu_y - a.enu_y))) % 360 - a.CourseOverGround) + 540) % 360 - 180) > 90)
                          AND
                          (abs((((360 + degrees(atan2(n2.enu_x - a.enu_x, n2.enu_y - a.enu_y))) % 360 - a.CourseOverGround) + 540) % 360 - 180) <= 90)

                    RETURN n1, n2
                    ORDER BY abs(sin(radians(abs((((360 + degrees(atan2(n2.enu_x - n1.enu_x, n2.enu_y - n1.enu_y))) % 360 - (360 + degrees(atan2(a.enu_x - n1.enu_x, a.enu_y - n1.enu_y))) % 360) + 540) % 360 - 180)))*point.distance(a.pos, n1.pos))/4 + (abs((((360 + degrees(atan2(n2.enu_x - n1.enu_x, n2.enu_y - n1.enu_y))) % 360 - a.CourseOverGround) + 540) % 360 - 180))/90 ASC
                    LIMIT 1
                }

                MERGE (a)-[:IS_ON {distance:point.distance(a.pos, n2.pos)}]->(n2)
                MERGE (a)-[:WAS_ON {distance:point.distance(a.pos, n1.pos)}]->(n1)

                WITH a, n1, n2

                MATCH (n1)-[n1_r2:HAS_NODE]-(w:OSMWay)-[n2_r1:HAS_NODE]-(n2)
                WHERE n1_r2.nd = n2_r1.nd + 1 OR n1_r2.nd = n2_r1.nd - 1
                MERGE (a)-[:IS_ON]-(w)
                
                RETURN true as updated_position
                
                UNION

                WITH a, n1, n2, w, n2_a, a_n1, a_w

                WITH a, n1, n2, w, n2_a, a_n1, a_w
                WHERE n1 IS NOT NULL
                      AND
                      n2 IS NOT NULL
                      AND
                      w IS NOT NULL
                      AND
                      abs(sin(radians(abs((((360 + degrees(atan2(n1.enu_x - n2.enu_x, n1.enu_y - n2.enu_y))) % 360 - (360 + degrees(atan2(a.enu_x - n2.enu_x, a.enu_y - n2.enu_y))) % 360) + 540) % 360 - 180)))*point.distance(a.pos, n2.pos)) <= 8
                      AND
                      (abs((((360 + degrees(atan2(n1.enu_x - a.enu_x, n1.enu_y - a.enu_y))) % 360 - a.CourseOverGround) + 540) % 360 - 180) > 90)
                
                MATCH (n2)-[n2_n1:NEXT_NODE]->(n1)-[n1_n3:NEXT_NODE]->(n3:OSMNode)-[:HAS_NODE]-(:OSMWay)-[:HAS_TAG]-(t:OSMTag {key:'highway'})
                WHERE NOT t.value IN $paths
                      AND
                      n2.ID <> n3.ID

                WITH a, n1, n3, n2_a, a_n1, a_w
                ORDER BY abs((((360 + degrees(atan2(n3.enu_x - n1.enu_x, n3.enu_y - n1.enu_y))) % 360 - a.CourseOverGround) + 540) % 360 - 180) ASC
                LIMIT 1

                MATCH (n1)-[n1_w:HAS_NODE]-(w:OSMWay)-[w_n3:HAS_NODE]-(n3)
                WHERE w_n3.nd = n1_w.nd + 1 OR w_n3.nd = n1_w.nd - 1

                DELETE n2_a, a_n1, a_w

                MERGE (a)-[:IS_ON]-(n3)
                MERGE (a)-[:WAS_ON]-(n1)
                MERGE (a)-[:IS_ON]-(w)

                RETURN true as updated_position

                UNION

                WITH a, n1, n2, w, n2_a, a_n1, a_w

                WITH a, n1, n2, w, n2_a, a_n1, a_w
                WHERE n1 IS NOT NULL
                      AND
                      n2 IS NOT NULL
                      AND
                      w IS NOT NULL
                      AND
                      abs(sin(radians(abs((((360 + degrees(atan2(n1.enu_x - n2.enu_x, n1.enu_y - n2.enu_y))) % 360 - (360 + degrees(atan2(a.enu_x - n2.enu_x, a.enu_y - n2.enu_y))) % 360) + 540) % 360 - 180)))*point.distance(a.pos, n2.pos)) <= 8
                      AND
                      (abs((((360 + degrees(atan2(n1.enu_x - a.enu_x, n1.enu_y - a.enu_y))) % 360 - a.CourseOverGround) + 540) % 360 - 180) <= 90)
                
                RETURN false as updated_position
           }

           RETURN updated_position
           ''', prop=prop, timestamp=timestamp, paths=utils.paths)

# Add connection data
def connection(tx, data, timestamp):
    # Get properties from data
    props = data.to_dict('records')

    # Add connection data
    tx.run('''
           MATCH (a:AutonomousVehicle:L4 {ID:11112000})

           UNWIND $props as prop

           MERGE (c:Connection:L4 {ID:prop.ID})
           SET c += prop, c.timestamp = $timestamp, C.pos = point({x:$prop.enu_x, y:$prop.enu_y})

           WITH c

           MERGE (c)-[:IS_CONNECTED]->(a)

           OPTIONAL MATCH (n2:OSMNode)-[n2_c:WAS_ON]-(c)-[c_n1:IS_ON]-(n1:OSMNode)
           OPTIONAL MATCH (c)-[c_w:IS_ON]-(w:OSMWay)

           WITH c, n1, n2, w, n2_c, c_n1, c_w

           CALL {
               WITH c, n1, n2, w, n2_c, c_n1, c_w

               WITH c, n1 as is_on, n2 as was_on, w as is_on_w, n2_c, c_n1, c_w
               WHERE (is_on IS NULL
                       AND
                       was_on IS NULL
                       AND
                       is_on_w IS NULL)
                       OR
                       abs(sin(radians(abs((((360 + degrees(atan2(is_on.enu_x - was_on.enu_x, is_on.enu_y - was_on.enu_y))) % 360 - (360 + degrees(atan2(c.enu_x - was_on.enu_x, c.enu_y - was_on.enu_y))) % 360) + 540) % 360 - 180)))*point.distance(c.pos, was_on.pos)) > 8
            
               DELETE n2_c, c_n1, c_w

               WITH c

               CALL {
                   WITH c

                   MATCH (n1:OSMNode)-[n1_r1:NEXT_NODE]->(n2:OSMNode), (n1)-[n1_r2:HAS_NODE]-(w:OSMWay)-[n2_r1:HAS_NODE]-(n2), (w:OSMWay)-[:HAS_TAG]-(t:OSMTag {key:'highway'})
                   WHERE (n1_r2.nd = n2_r1.nd + 1 OR n1_r2.nd = n2_r1.nd - 1)
                           AND
                           (c.type IN $non_motor_conn OR (c.type IN $motor_conn AND NOT t.value IN $paths))
                           AND
                           (abs((((360 + degrees(atan2(n1.enu_x - c.enu_x, n1.enu_y - c.enu_y))) % 360 - c.CourseOverGround) + 540) % 360 - 180) > 90)
                           AND
                           (abs((((360 + degrees(atan2(n2.enu_x - c.enu_x, n2.enu_y - c.enu_y))) % 360 - c.CourseOverGround) + 540) % 360 - 180) <= 90)

                   RETURN n1, n2
                   ORDER BY abs(sin(radians(abs((((360 + degrees(atan2(n2.enu_x - n1.enu_x, n2.enu_y - n1.enu_y))) % 360 - (360 + degrees(atan2(c.enu_x - n1.enu_x, c.enu_y - n1.enu_y))) % 360) + 540) % 360 - 180)))*point.distance(c.pos, n1.pos))/4 + (abs((((360 + degrees(atan2(n2.enu_x - n1.enu_x, n2.enu_y - n1.enu_y))) % 360 - c.CourseOverGround) + 540) % 360 - 180))/90 ASC
                   LIMIT 1
               }

               MERGE (c)-[:IS_ON {distance:point.distance(a.pos, n2.pos)}]->(n2)
               MERGE (c)-[:WAS_ON {distance:point.distance(a.pos, n1.pos)}]->(n1)

               WITH c, n1, n2

               MATCH (n1)-[n1_r2:HAS_NODE]-(w:OSMWay)-[n2_r1:HAS_NODE]-(n2)
               WHERE n1_r2.nd = n2_r1.nd + 1 OR n1_r2.nd = n2_r1.nd - 1
               MERGE (c)-[:IS_ON]-(w)
            
               RETURN true as updated_position
            
               UNION

               WITH c, n1, n2, w, n2_c, c_n1, c_w

               WITH c, n1, n2, w, n2_c, c_n1, c_w
               WHERE n1 IS NOT NULL
                       AND
                       n2 IS NOT NULL
                       AND
                       w IS NOT NULL
                       AND
                       abs(sin(radians(abs((((360 + degrees(atan2(n1.enu_x - n2.enu_x, n1.enu_y - n2.enu_y))) % 360 - (360 + degrees(atan2(c.enu_x - n2.enu_x, c.enu_y - n2.enu_y))) % 360) + 540) % 360 - 180)))*point.distance(c.pos, n2.pos)) <= 8
                       AND
                       (abs((((360 + degrees(atan2(n1.enu_x - c.enu_x, n1.enu_y - c.enu_y))) % 360 - c.CourseOverGround) + 540) % 360 - 180) > 90)
            
               MATCH (n2)-[n2_n1:NEXT_NODE]->(n1)-[n1_n3:NEXT_NODE]->(n3:OSMNode)-[:HAS_NODE]-(:OSMWay)-[:HAS_TAG]-(t:OSMTag {key:'highway'})
               WHERE (c.type IN $non_motor_conn OR (c.type IN $motor_conn AND NOT t.value IN $paths))
                       AND
                       n2.ID <> n3.ID

               WITH c, n1, n3, n2_c, c_n1, c_w
               ORDER BY abs((((360 + degrees(atan2(n3.enu_x - n1.enu_x, n3.enu_y - n1.enu_y))) % 360 - c.CourseOverGround) + 540) % 360 - 180) ASC
               LIMIT 1

               MATCH (n1)-[n1_w:HAS_NODE]-(w:OSMWay)-[w_n3:HAS_NODE]-(n3)
               WHERE w_n3.nd = n1_w.nd + 1 OR w_n3.nd = n1_w.nd - 1
   
               DELETE n2_c, c_n1, c_w

               MERGE (c)-[:IS_ON]-(n3)
               MERGE (c)-[:WAS_ON]-(n1)
               MERGE (c)-[:IS_ON]-(w)

               RETURN true as updated_position

               UNION

               WITH c, n1, n2, w, n2_c, c_n1, c_w

               WITH c, n1, n2, w, n2_c, c_n1, c_w
               WHERE n1 IS NOT NULL
                       AND
                       n2 IS NOT NULL
                       AND
                       w IS NOT NULL
                       AND
                       abs(sin(radians(abs((((360 + degrees(atan2(n1.enu_x - n2.enu_x, n1.enu_y - n2.enu_y))) % 360 - (360 + degrees(atan2(c.enu_x - n2.enu_x, c.enu_y - n2.enu_y))) % 360) + 540) % 360 - 180)))*point.distance(c.pos, n2.pos)) <= 8
                       AND
                       (abs((((360 + degrees(atan2(n1.enu_x - c.enu_x, n1.enu_y - c.enu_y))) % 360 - c.CourseOverGround) + 540) % 360 - 180) <= 90)
            
               RETURN false as updated_position
           }

           RETURN updated_position
           ''', props=props, timestamp=timestamp, paths=utils.paths,
           motor_conn=utils.motor_conn, non_motor_conn=utils.non_motor_conn)

# Add detection data
def detection(tx, data, timestamp):
    # Set the maximum number of allowed prediction steps
    pred_steps = 2

    # Refactor graph by deleting old detection/prediction data
    tx.run('''
           OPTIONAL MATCH (d:Detection)
           WHERE d.pred_steps > $pred_steps
           OPTIONAL MATCH (p:Prediction)
           WHERE p.pred_steps > $pred_steps + 1
           DETACH DELETE d, p
           ''', pred_steps=pred_steps)

    # Get previous detection data
    result = tx.run('''
                    OPTIONAL MATCH (p:Prediction)
                    WITH collect(p.class) as class,
                         collect(p.exist_prob) as exist_prob,
                         collect(p.RelativeVelocity) as RelativeVelocity,
                         collect(p.dx) as dx,
                         collect(p.phi_r) as phi_r,
                         collect(p.phi_l) as phi_l,
                         collect(p.handle) as handle,
                         collect(p.Smjr_ellipse_orient_n) as Smjr_ellipse_orient_n,
                         collect(p.Smjr_ax_ellipse) as Smjr_ax_ellipse,
                         collect(p.Smnr_ax_ellipse) as Smnr_ax_ellipse,
                         collect(p.Lat) as Lat,
                         collect(p.Lon) as Lon,
                         collect(p.ecef_x) as ecef_x,
                         collect(p.ecef_y) as ecef_y,
                         collect(p.ecef_z) as ecef_z,
                         collect(p.CourseOverGround) as CourseOverGround,
                         collect(p.SpeedKMH) as SpeedKMH,
                         collect(p.type) as type,
                         collect(p.refID*0 + 22224000) as refID,
                         collect(toFloat(duration.between(p.timestamp, $timestamp).milliseconds)/1000) as delay,
                         collect(p.ID) as ID,
                         collect(p.enu_x) as enu_x,
                         collect(p.enu_y) as enu_y,
                         collect(p.enu_z) as enu_z,
                         collect(p.a) as a,
                         collect(p.omega) as omega,
                         collect(p.m) as m,
                         collect(p.P) as P,
                         collect(p.pred_steps) as pred_steps
                    RETURN apoc.map.fromLists(
                    ['class', 'exist_prob', 'RelativeVelocity', 'dx', 'phi_r',
                    'phi_l', 'handle', 'Smjr_ellipse_orient_n',
                    'Smjr_ax_ellipse', 'Smnr_ax_ellipse', 'Lat', 'Lon',
                    'ecef_x', 'ecef_y', 'ecef_z',
                    'CourseOverGround', 'SpeedKMH', 'type', 'refID', 'delay',
                    'ID', 'enu_x', 'enu_y', 'enu_z', 'a', 'omega', 'm', 'P', 'pred_steps'],
                    [class, exist_prob, RelativeVelocity, dx, phi_r, phi_l,
                    handle, Smjr_ellipse_orient_n, Smjr_ax_ellipse,
                    Smnr_ax_ellipse, Lat, Lon, ecef_x, ecef_y, ecef_z,
                    CourseOverGround, SpeedKMH, type,
                    refID, delay, ID, enu_x, enu_y, enu_z, a, omega, m, P, pred_steps])
                    ''', timestamp=timestamp)

    predicted_detections = result.single().value()

    # Check wether there is data to process
    if not data.empty or bool([field for field in predicted_detections.values() if field != []]):
        # Match the detections and predict their next state
        (det_props, pred_props) = utils.match(data, predicted_detections)

        # Check wether there is data

        props = [[det, pred] for det, pred in zip(det_props, pred_props)]

        # Add detection and prediction data
        tx.run('''
               MATCH (a:AutonomousVehicle:L4 {ID:11112000})

               UNWIND $props as prop

               MERGE (d:Detection:L5 {ID:prop[0].ID})
               SET d += prop[0], d.timestamp = $timestamp, d.pos = point({x:prop[0].enu_x, y:prop[0].enu_y})
               MERGE (d)-[dr:IS_DETECTED]->(a)
               SET dr.type = prop[0].type

               MERGE (p:Prediction:L6 {ID:prop[1].ID})
               SET p += prop[1], p.timestamp = $next_timestamp
               MERGE (p)-[pr:IS_PREDICTED]->(a)
               SET pr.type = prop[1].type
               MERGE (d)-[:WILL_BE]-(p)
                
               WITH d

               OPTIONAL MATCH (n2:OSMNode)-[n2_d:WAS_ON]-(d)-[d_n1:IS_ON]-(n1:OSMNode)
               OPTIONAL MATCH (d)-[d_w:IS_ON]-(w:OSMWay)

               WITH d, n1, n2, w, n2_d, d_n1, d_w

               CALL {
                    WITH d, n1, n2, w, n2_d, d_n1, d_w

                    WITH d, n1 as is_on, n2 as was_on, w as is_on_w, n2_d, d_n1, d_w
                    WHERE (is_on IS NULL
                           AND
                           was_on IS NULL
                           AND
                           is_on_w IS NULL)
                          OR
                          abs(sin(radians(abs((((360 + degrees(atan2(is_on.enu_x - was_on.enu_x, is_on.enu_y - was_on.enu_y))) % 360 - (360 + degrees(atan2(d.enu_x - was_on.enu_x, d.enu_y - was_on.enu_y))) % 360) + 540) % 360 - 180)))*point.distance(d.pos, was_on.pos)) > 8

                    DELETE n2_d, d_n1, d_w

                    WITH d

                    CALL {
                        WITH d

                        MATCH (n1:OSMNode)-[n1_r1:NEXT_NODE]->(n2:OSMNode), (n1)-[n1_r2:HAS_NODE]-(w:OSMWay)-[n2_r1:HAS_NODE]-(n2), (w:OSMWay)-[:HAS_TAG]-(t:OSMTag {key:'highway'})
                        WHERE (n1_r2.nd = n2_r1.nd + 1 
                               OR 
                               n1_r2.nd = n2_r1.nd - 1)
                              AND
                              (d.type IN $non_motor_det OR (d.type IN $motor_det AND NOT t.value IN $paths))
                              AND
                              (abs((((360 + degrees(atan2(n1.enu_x - d.enu_x, n1.enu_y - d.enu_y))) % 360 - d.CourseOverGround) + 540) % 360 - 180) > 90)
                              AND
                              (abs((((360 + degrees(atan2(n2.enu_x - d.enu_x, n2.enu_y - d.enu_y))) % 360 - d.CourseOverGround) + 540) % 360 - 180) <= 90)
                                
                        RETURN n1, n2
                        ORDER BY abs(sin(radians(abs((((360 + degrees(atan2(n2.enu_x - n1.enu_x, n2.enu_y - n1.enu_y))) % 360 - (360 + degrees(atan2(d.enu_x - n1.enu_x, d.enu_y - n1.enu_y))) % 360) + 540) % 360 - 180)))*point.distance(d.pos, n1.pos))/4 + (abs((((360 + degrees(atan2(n2.enu_x - n1.enu_x, n2.enu_y - n1.enu_y))) % 360 - d.CourseOverGround) + 540) % 360 - 180))/90 ASC
                        LIMIT 1
                    }

                    MERGE (d)-[:IS_ON {distance:point.distance(d.pos, n2.pos)}]->(n2)
                    MERGE (d)-[:WAS_ON {distance:point.distance(d.pos, n1.pos)}]->(n1)

                    WITH d, n1, n2

                    MATCH (n1)-[n1_r2:HAS_NODE]-(w:OSMWay)-[n2_r1:HAS_NODE]-(n2)
                    WHERE n1_r2.nd = n2_r1.nd + 1 OR n1_r2.nd = n2_r1.nd - 1
                    MERGE (d)-[:IS_ON]-(w)

                    RETURN true as updated_position

                    UNION

                    WITH d, n1, n2, w, n2_d, d_n1, d_w

                    WITH d, n1, n2, w, n2_d, d_n1, d_w
                    WHERE n1 IS NOT NULL
                          AND
                          n2 IS NOT NULL
                          AND
                          w IS NOT NULL
                          AND
                          (abs(sin(radians(abs((((360 + degrees(atan2(n1.enu_x - n2.enu_x, n1.enu_y - n2.enu_y))) % 360 - (360 + degrees(atan2(d.enu_x - n2.enu_x, d.enu_y - n2.enu_y))) % 360) + 540) % 360 - 180)))*point.distance(d.pos, n2.pos)) <= 8)
                          AND
                          (abs((((360 + degrees(atan2(n1.enu_x - d.enu_x, n1.enu_y - d.enu_y))) % 360 - d.CourseOverGround) + 540) % 360 - 180) > 90)
                    
                    MATCH (n2)-[n2_n1:NEXT_NODE]->(n1)-[n1_n3:NEXT_NODE]->(n3:OSMNode)-[:HAS_NODE]-(:OSMWay)-[:HAS_TAG]-(t:OSMTag {key:'highway'})
                    WHERE (d.type IN $non_motor_det OR (d.type IN $motor_det AND NOT t.value IN $paths))
                          AND
                          n2.ID <> n3.ID

                    WITH d, n1, n3, n2_d, d_n1, d_w
                    ORDER BY abs((((360 + degrees(atan2(n3.enu_x - n1.enu_x, n3.enu_y - n1.enu_y))) % 360 - d.CourseOverGround) + 540) % 360 - 180) ASC
                    LIMIT 1

                    MATCH (n1)-[n1_w:HAS_NODE]-(w:OSMWay)-[w_n3:HAS_NODE]-(n3)
                    WHERE w_n3.nd = n1_w.nd + 1 OR w_n3.nd = n1_w.nd - 1

                    DELETE n2_d, d_n1, d_w

                    MERGE (d)-[:IS_ON]-(n3)
                    MERGE (d)-[:WAS_ON]-(n1)
                    MERGE (d)-[:IS_ON]-(w)

                    RETURN true as updated_position

                    UNION

                    WITH d, n1, n2, w, n2_d, d_n1, d_w

                    WITH d, n1, n2, w, n2_d, d_n1, d_w
                    WHERE n1 IS NOT NULL
                          AND
                          n2 IS NOT NULL
                          AND
                          w IS NOT NULL
                          AND
                          (abs(sin(radians(abs((((360 + degrees(atan2(n1.enu_x - n2.enu_x, n1.enu_y - n2.enu_y))) % 360 - (360 + degrees(atan2(d.enu_x - n2.enu_x, d.enu_y - n2.enu_y))) % 360) + 540) % 360 - 180)))*point.distance(d.pos, n2.pos)) <= 8)
                          AND
                          (abs((((360 + degrees(atan2(n1.enu_x - d.enu_x, n1.enu_y - d.enu_y))) % 360 - d.CourseOverGround) + 540) % 360 - 180) <= 90)
                    
                    RETURN false as updated_position
               }

               RETURN updated_position
               ''', props=props, timestamp=timestamp,
               next_timestamp=timestamp + timedelta(milliseconds=100),
               paths=utils.paths, motor_det=utils.motor_det,
               non_motor_det=utils.non_motor_det)

        # Get the possible nodes one entity might go to
        result = tx.run('''
                        MATCH (d:Detection)
                        WHERE d.SpeedKMH <> 0

                        MATCH (d)-[:WAS_ON]-(n0:OSMNode)
                        MATCH (d)-[d_r1:IS_ON]-(n1:OSMNode)
                        MATCH (d)-[:IS_ON]-(w0:OSMWay)

                        MATCH (n0)-[:NEXT_NODE]->(n1)-[n1_r1:NEXT_NODE]->(n2:OSMNode)-[:HAS_NODE]-(w:OSMWay)-[:HAS_TAG]-(t:OSMTag {key:'highway'})
                        WHERE (d.type IN $non_motor_det OR (d.type IN $motor_det AND NOT t.value IN $paths))
                              AND
                              n0.ID <> n2.ID

                        WITH d, n0, n1, n2, w0

                        MATCH (n1)-[n1_r2:HAS_NODE]-(w1:OSMWay)-[n2_r1:HAS_NODE]-(n2)
                        WHERE n1_r2.nd = n2_r1.nd + 1 OR n1_r2.nd = n2_r1.nd - 1

                        OPTIONAL MATCH (w1)-[:HAS_TAG]-(t:OSMTag {key:'maxspeed'})

                        WITH DISTINCT d, n0, n1, n2, w0, w1, t
                        RETURN d.ID as id,
                               d.ecef_x as ecef_x,
                               d.ecef_y as ecef_y,
                               d.ecef_z as ecef_z,
                               d.SpeedKMH/3.6 AS v0,
                               d.a AS a0,
                               d.CourseOverGround as psi0,
                               w0.ID as w0_id,
                               n1.ID as n1_id,
                               n1.ecef_x as n1_ecef_x,
                               n1.ecef_y as n1_ecef_y,
                               n1.ecef_z as n1_ecef_z,
                               collect(n2.ID) as n2_ids,
                               collect(n2.ecef_x) as n2_ecef_xs,
                               collect(n2.ecef_y) as n2_ecef_ys,
                               collect(n2.ecef_z) as n2_ecef_zs,
                               collect(w1.ID) as w1_ids,
                               collect(CASE t.value
                               WHEN NULL THEN 50/3.6
                               ELSE toFloat(t.value)
                               END) as speed_limits
                        ''', paths=utils.paths, motor_det=utils.motor_det,
                        non_motor_det=utils.non_motor_det)

        possible_actions = [[record['id'],
                             record['ecef_x'],
                             record['ecef_y'],
                             record['ecef_z'],
                             record['v0'],
                             record['a0'],
                             record['psi0'],
                             record['w0_id'],
                             record['n1_id'],
                             record['n1_ecef_x'],
                             record['n1_ecef_y'],
                             record['n1_ecef_z'],
                             record['n2_ids'],
                             record['n2_ecef_xs'],
                             record['n2_ecef_ys'],
                             record['n2_ecef_zs'],
                             record['w1_ids'],
                             record['speed_limits']]
                            for record in result]

        likely_actions, likely_clothoids = utils.predict_action(possible_actions)

        # Get the possible collisions with an entity
        result = tx.run('''
                        MATCH (n1:OSMNode)-[a_r1:IS_ON]-(a:AutonomousVehicle {ID:11112000})-[:IS_ON]-(w1:OSMWay)
                        WHERE a.VehSpdMPS <> 0

                        MATCH (n1)-[n1_r1:HAS_NODE]-(w1)

                        UNWIND $likely_actions as likely_action

                        MATCH (w0:OSMWay {ID:likely_action[4]})-[:HAS_NODE]-(n2:OSMNode {ID:likely_action[5]})-[n2_r1:HAS_NODE]-(w2:OSMWay {ID:likely_action[7]})-[n3_r1:HAS_NODE]-(n3:OSMNode {ID:likely_action[6]})

                        CALL {
                            WITH w0, w1, w2, n2_r1, n3_r1

                            WITH w1, w2, n2_r1, n3_r1
                            WHERE w0.ID <> w1.ID
                                  AND
                                  w1.ID <> w2.ID

                            MATCH (w1)-[n4_r1:HAS_NODE]-(n4)-[n4_r2:HAS_NODE]-(w2)
                            WHERE sign(n2_r1.nd - n3_r1.nd) = sign(n3_r1.nd - n4_r2.nd)
                                  OR
                                  n4_r2 IN [n2_r1, n3_r1]

                            RETURN n4, n4_r1, n4_r2
                            LIMIT 1
                        }

                        CALL {
                            WITH w1, n1_r1, n4_r1

                            MATCH (w1)-[w1_r1:HAS_NODE]-(a_ns:OSMNode)
                            WHERE n1_r1.nd <= w1_r1.nd <= n4_r1.nd
                                  OR
                                  n1_r1.nd >= w1_r1.nd >= n4_r1.nd

                            RETURN a_ns
                            ORDER BY sign(n4_r1.nd - n1_r1.nd)*w1_r1.nd
                        }

                        CALL {
                            WITH w2, n2_r1, n4_r2

                            MATCH (w2)-[w2_r1:HAS_NODE]-(d_ns:OSMNode)
                            WHERE n2_r1.nd <= w2_r1.nd <= n4_r2.nd
                                  OR
                                  n2_r1.nd >= w2_r1.nd >= n4_r2.nd

                            RETURN d_ns
                            ORDER BY sign(n4_r2.nd - n2_r1.nd)*w2_r1.nd
                        }

                        OPTIONAL MATCH (w1)-[:HAS_TAG]-(t1:OSMTag {key:'maxspeed'})
                        OPTIONAL MATCH (w2)-[:HAS_TAG]-(t2:OSMTag {key:'maxspeed'})

                        RETURN [a.ecef_x, a.ecef_y, a.ecef_z] as a_s0,
                               a.VehSpdMPS as a_v0,
                               a.LonAccel as a_a0,
                               a.CourseOverGround as a_psi0,
                               collect(distinct [a_ns.ecef_x,
                                                 a_ns.ecef_y,
                                                 a_ns.ecef_z]) as a_nodes,
                               CASE t1.value
                               WHEN NULL THEN 50/3.6
                               ELSE toFloat(t1.value)
                               END as a_speed_limit,
                               likely_action[8] as s0,
                               likely_action[1] as v0,
                               likely_action[2] as a0,
                               likely_action[3] as psi0,
                               collect(distinct [d_ns.ecef_x,
                                                 d_ns.ecef_y,
                                                 d_ns.ecef_z]) as nodes,
                               CASE t2.value
                               WHEN NULL THEN 50/3.6
                               ELSE toFloat(t2.value)
                               END as speed_limit
                        ''', likely_actions=likely_actions)

        possible_collisions = [[record['a_s0'],
                                record['a_v0'],
                                record['a_a0'],
                                record['a_psi0'],
                                record['a_nodes'],
                                record['a_speed_limit'],
                                record['s0'],
                                record['v0'],
                                record['a0'],
                                record['psi0'],
                                record['nodes'],
                                record['speed_limit']]
                               for record in result]

        ((best_mp, mp_type),
         coll_trajs) = utils.predict_collision(possible_collisions)

        # Return the best motion primitive to avoid collisions, if any, and the
        # detection likely clothoid
        return best_mp, mp_type, coll_trajs, likely_clothoids
    else:
        return np.empty(0), None, [], []
