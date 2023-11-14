"""--------------------------------------------------------------------------
    This file is meant to handle all things related to the Neo4j database.
   --------------------------------------------------------------------------"""

# Remember that with the community edition the only way to query is via the
# interpreted runtime, the slowest!

''' IMPORTS '''
# warnings to suppress the ExperimentalWarning
import warnings

# GraphDatabase to connect to the database and ExperimentalWarning to suppress
# them
from neo4j import GraphDatabase, ExperimentalWarning

# format_labels and format_properties to format them
from ldm.utils import format_labels, format_properties

''' FUNCTIONS '''
# Open the connection to the database
def connect_db():
    # Connect to the database
    uri    = 'bolt://localhost:7687'
    driver = GraphDatabase.driver(uri, auth=('usr', 'pw'))

    # Test the connection
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", category=ExperimentalWarning)

        driver.verify_connectivity()

    return driver

# Close the connection to the database
def close_db(driver):
    # Close the connection
    driver.close()

# Export the entire database
def export_db(driver, statement, fileName):
    # Query
    def query(tx, statement, fileName):
        tx.run('''
               CALL apoc.export.cypher.query(
               $statement,
               $fileName,
               {writeNodeProperties:true,
               format:'cypher-shell',
               cypherFormat:'updateAll',
               ifNotExists:true}
               )
               ''', statement=statement, fileName=fileName)

    # Run query
    with driver.session() as session:
        session.write_transaction(query, statement, fileName)

# Import the entire database
def import_db(driver, fileName):
    # Query
    def query(tx, fileName):
        tx.run('''
               CALL apoc.cypher.runFile($fileName)
               ''', fileName=fileName)

    # Run query
    with driver.session() as session:
        session.write_transaction(query, fileName)

# Export the entire database
def export_graphml(driver, timestamp):
    # Create the filename
    filename = 'ldm_' + timestamp + '.graphml'

    # Query
    def query(tx):
        tx.run('''
               CALL apoc.export.graphml.all($filename,
               {format:"gephi", useTypes:true})
               ''', filename=filename)

    # Run query
    with driver.session() as session:
        session.write_transaction(query)

# Delete all data from the database
def empty_db(driver):
    # Query
    def query(tx):
        tx.run('''
               MATCH (n)
               DETACH DELETE n
               ''')

    # Run query
    with driver.session() as session:
        session.write_transaction(query)

# Create uniqueness constraints
def create_uniqueness_constraints(driver):
    # Query
    def query(tx):
        # Create uniqueness constraint on the node id
        tx.run('''
               CREATE CONSTRAINT osm_node IF NOT EXISTS
               FOR (n:OSMNode)
               REQUIRE n.ID IS UNIQUE
               ''')
        # Create uniqueness constraint on the way id
        tx.run('''
               CREATE CONSTRAINT osm_way IF NOT EXISTS
               FOR (w:OSMWay)
               REQUIRE w.ID IS UNIQUE
               ''')
        # Create uniqueness constraint on the relation id
        tx.run('''
               CREATE CONSTRAINT osm_relation IF NOT EXISTS
               FOR (r:OSMRelation)
               REQUIRE r.ID IS UNIQUE
               ''')
        # Create uniqueness constraint on the tag key-value pair
        tx.run('''
               CREATE CONSTRAINT osm_tag IF NOT EXISTS
               FOR (t:OSMTag)
               REQUIRE (t.key, t.value) IS UNIQUE
               ''')
        # Create uniqueness constraint on the phase id
        tx.run('''
               CREATE CONSTRAINT signal_phase IF NOT EXISTS
               FOR (s:SignalPhase)
               REQUIRE s.ID IS UNIQUE
               ''')
        # Create uniqueness constraint on the autonomous vehicle id
        tx.run('''
               CREATE CONSTRAINT autonomous_vehicle IF NOT EXISTS
               FOR (a:AutonomousVehicle)
               REQUIRE a.ID IS UNIQUE
               ''')
        # Create uniqueness constraint on the roadside unit id
        tx.run('''
               CREATE CONSTRAINT roadside_unit IF NOT EXISTS
               FOR (r:RoadSideUnit)
               REQUIRE r.ID IS UNIQUE
               ''')
        # Create uniqueness constraint on the detection id
        tx.run('''
               CREATE CONSTRAINT detection IF NOT EXISTS
               FOR (d:Detection)
               REQUIRE d.ID IS UNIQUE
               ''')
        # Create uniqueness constraint on the prediction id
        tx.run('''
               CREATE CONSTRAINT prediction IF NOT EXISTS
               FOR (p:Prediction)
               REQUIRE p.ID IS UNIQUE
               ''')

    # Run query
    with driver.session() as session:
        session.write_transaction(query)

# Dynamically merge a node
def merge_node(driver, label, identProps, onCreateProps={}, onMatchProps={}):
    # Query
    def query(tx, label, identProps, onCreateProps, onMatchProps):
        labels = format_labels(label)
        props  = format_properties(identProps, 'identProps')

        cypher = '''
                 MERGE (n:'''+labels+''' {'''+props+'''})
                 ON CREATE SET n += $onCreateProps
                 ON MATCH SET n += $onMatchProps
                 RETURN n
                 '''

        tx.run(cypher, identProps=identProps, onCreateProps=onCreateProps,
               onMatchProps=onMatchProps)

    # Run query
    with driver.session() as session:
        session.write_transaction(query, label, identProps, onCreateProps,
                                  onMatchProps)

# Dynamically merge a relationship
def merge_relationship(driver, startNode, relationshipType, endNode,
                       identProps={}, onCreateProps={}, onMatchProps={}):
    # Query
    def query(tx, startNode, relationshipType, endNode, identProps={},
              onCreateProps={}, onMatchProps={}):
        n1Labels     = format_labels(startNode[0])
        n1IdentProps = format_properties(startNode[1], 'n1IdentProps')
        n2Labels     = format_labels(endNode[0])
        n2IdentProps = format_properties(endNode[1], 'n2IdentProps')
        props        = format_properties(identProps, 'identProps')

        cypher = '''
                 MATCH (n1:'''+n1Labels+''' {'''+n1IdentProps+'''})
                 MATCH (n2:'''+n2Labels+''' {'''+n2IdentProps+'''})
                 MERGE (n1)-[r:'''+relationshipType+''' {'''+props+'''}]->(n2)
                 ON CREATE SET r += $onCreateProps
                 ON MATCH SET r += $onMatchProps
                 RETURN n1, n2
                 '''

        tx.run(cypher, n1IdentProps=startNode[1], n2IdentProps=endNode[1],
               identProps=identProps, onCreateProps=onCreateProps,
               onMatchProps=onMatchProps)

    # Run query
    with driver.session() as session:
        session.write_transaction(query, startNode, relationshipType, endNode,
                                  identProps, onCreateProps, onMatchProps)

# Dynamnically match a node
def match_node(driver, label, identProps):
    # Query
    def query(tx, label, identProps):
        labels = format_labels(label)
        props  = format_properties(identProps, 'identProps')

        cypher = 'MATCH (n:'+labels+' {'+props+'}) RETURN properties(n)'

        result = tx.run(cypher, identProps=identProps)
        nodes  = []

        for record in result:
            nodes.append(record.value())

        return nodes

    # Run query
    with driver.session() as session:
        return session.read_transaction(query, label, identProps)

# Dynamically match nodes one relationship from the target
def match_relationship_node(driver, label, identProps, relationshipType):
    # Query
    def query(tx, label, identProps, relationshipType):
        labels = format_labels(label)
        props  = format_properties(identProps, 'identProps')

        cypher = '''
        MATCH (n:'''+labels+''' {'''+props+'''})-[r:'''+relationshipType+''']-(m)
        RETURN type(r), properties(r), labels(m), properties(m)
        '''

        result  = tx.run(cypher, identProps=identProps)
        nodes = []

        for record in result:
            nodes.append(record.values())

        return nodes

    # Run query
    with driver.session() as session:
        return session.read_transaction(query, label, identProps, relationshipType)

# Relate to the closest node based on geographical distance
def relate_to_closest_node(driver, label, identProps):
    # Query
    def query(tx, label, identProps):
        n1Labels     = format_labels(label)
        n1IdentProps = format_properties(identProps, 'n1IdentProps')

        tx.run('''
               MATCH (n1:'''+n1Labels+''' {'''+n1IdentProps+'''})
               OPTIONAL MATCH (n1)-[r:IS_ON]-()
               DELETE r
               WITH n1,
               point({latitude:n1.Lat, longitude:n1.Lon}) as pos
               CALL {
                   WITH n1, pos
                   MATCH (n2:OSMNode)
                   WHERE point.distance(pos, point({latitude:n2.Lat, longitude:n2.Lon})) < 50
                   RETURN n2, point({latitude:n2.Lat, longitude:n2.Lon}) as pos2
                   ORDER BY point.distance(pos, point({latitude:n2.Lat, longitude:n2.Lon})) ASC
                   LIMIT 1
               }
               WITH n1, pos, n2, pos2
               MERGE (n1)-[:IS_ON {distance:point.distance(pos,
               pos2)}]->(n2)
               ''', n1IdentProps=identProps)

    # Run query
    with driver.session() as session:
        session.write_transaction(query, label, identProps)
