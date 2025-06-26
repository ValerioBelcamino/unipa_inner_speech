from neo4j import GraphDatabase
from langchain_neo4j import Neo4jGraph

class RehabilitationDB:
    def __init__(self, uri, user, password):
        self.driver = GraphDatabase.driver(uri, auth=(user, password))
        self.graph = Neo4jGraph(uri, user, password)
        print(self.graph.schema)

    def close(self):
        self.driver.close()

    def clear_database(self):
        with self.driver.session() as session:
            session.run("MATCH (n) DETACH DELETE n")

    def create_exercises(self):
        exercises = [
            {"name": "sollevamento braccia alternato", "num_repetitions": 15, "balance": False, "target_joint": "spalla"},
            {"name": "stretching arti inferiori", "num_repetitions": 8, "balance": True, "target_joint": "ginocchio"},
            {"name": "contrazione scapole", "num_repetitions": 12, "balance": False, "target_joint": "spalla"},
            {"name": "camminata tallone-punta", "num_repetitions": 20, "balance": True, "target_joint": "caviglia"},
            {"name": "torsione spinale seduta", "num_repetitions": 10, "balance": False, "target_joint": "colonna"},
            {"name": "stretching flessori anca", "num_repetitions": 6, "balance": True, "target_joint": "anca"},
            {"name": "sollevamento polpacci", "num_repetitions": 15, "balance": True, "target_joint": "caviglia"},
            {"name": "cerchi con le braccia", "num_repetitions": 12, "balance": False, "target_joint": "spalla"},
            {"name": "rinforzo quadricipiti", "num_repetitions": 10, "balance": False, "target_joint": "ginocchio"},
            {"name": "rotazioni collo", "num_repetitions": 8, "balance": False, "target_joint": "collo"},
            {"name": "equilibrio su una gamba", "num_repetitions": 5, "balance": True, "target_joint": "caviglia"},
            {"name": "flessione polso", "num_repetitions": 15, "balance": False, "target_joint": "polso"},
            {"name": "ponte glutei", "num_repetitions": 12, "balance": False, "target_joint": "anca"},
            {"name": "pompe caviglie", "num_repetitions": 20, "balance": False, "target_joint": "caviglia"},
            {"name": "flessioni al muro", "num_repetitions": 10, "balance": False, "target_joint": "spalla"}
        ]

        with self.driver.session() as session:
            for i, exercise in enumerate(exercises):
                session.run(
                    "CREATE (e:Exercise {name: $name, num_repetitions: $num_repetitions, "
                    "balance: $balance, target_joint: $target_joint, id: $id})",
                    name=exercise["name"],
                    num_repetitions=exercise["num_repetitions"],
                    balance=exercise["balance"],
                    target_joint=exercise["target_joint"],
                    id=i+1
                )

    def create_exercise_steps(self):
        exercise_steps = {
            1: [  # sollevamento braccia alternato
                {"name": "posizione iniziale", "description": "stai in piedi con i piedi alla larghezza delle spalle, braccia ai lati.", "duration": 2},
                {"name": "alza braccio destro", "description": "solleva il braccio destro in avanti fino all'altezza della spalla con gomito dritto.", "duration": 3},
                {"name": "abbassa braccio destro", "description": "abbassa lentamente il braccio destro alla posizione iniziale.", "duration": 2},
                {"name": "alza braccio sinistro", "description": "solleva il braccio sinistro in avanti fino all'altezza della spalla con gomito dritto.", "duration": 3},
                {"name": "abbassa braccio sinistro", "description": "abbassa lentamente il braccio sinistro alla posizione iniziale.", "duration": 2}
            ],
            2: [  # stretching arti inferiori
                {"name": "posizione seduta", "description": "siediti sul bordo della sedia con la schiena dritta e i piedi appoggiati.", "duration": 3},
                {"name": "estendi gamba destra", "description": "raddrizza la gamba destra e fletti il piede verso l'alto.", "duration": 15},
                {"name": "ritorna gamba destra", "description": "abbassa lentamente la gamba destra alla posizione iniziale.", "duration": 3},
                {"name": "estendi gamba sinistra", "description": "raddrizza la gamba sinistra e fletti il piede verso l'alto.", "duration": 15}
            ],
            3: [  # contrazione scapole
                {"name": "posizione eretta", "description": "stai in piedi con le braccia rilassate ai lati.", "duration": 2},
                {"name": "stringi scapole", "description": "stringi le scapole insieme mantenendo le braccia dritte.", "duration": 5},
                {"name": "mantieni posizione", "description": "mantieni la contrazione respirando normalmente.", "duration": 3},
                {"name": "rilascia", "description": "rilascia lentamente le scapole alla posizione iniziale.", "duration": 2}
            ],
            4: [  # camminata tallone-punta
                {"name": "posizione di partenza", "description": "stai in piedi con il piede destro direttamente davanti al sinistro.", "duration": 3},
                {"name": "passo avanti", "description": "posiziona il tallone sinistro direttamente davanti alla punta destra.", "duration": 2},
                {"name": "continua camminata", "description": "ripeti i passi tallone-punta mantenendo l'equilibrio.", "duration": 2}
            ],
            5: [  # torsione spinale seduta
                {"name": "posizione seduta", "description": "siediti dritto con i piedi appoggiati e le mani sulle spalle.", "duration": 3},
                {"name": "ruota a destra", "description": "ruota il busto verso destra mantenendo i fianchi rivolti in avanti.", "duration": 5},
                {"name": "ritorna al centro", "description": "ritorna lentamente alla posizione iniziale.", "duration": 2},
                {"name": "ruota a sinistra", "description": "ruota il busto verso sinistra mantenendo i fianchi rivolti in avanti.", "duration": 5}
            ],
            6: [  # stretching flessori anca
                {"name": "posizione affondo", "description": "fai un passo avanti con il piede destro in posizione di affondo.", "duration": 3},
                {"name": "abbassa fianchi", "description": "abbassa delicatamente i fianchi mantenendo il ginocchio davanti sopra la caviglia.", "duration": 20},
                {"name": "cambia gamba", "description": "cambia alla posizione con il piede sinistro avanti.", "duration": 3},
                {"name": "stretching sinistro", "description": "abbassa i fianchi per allungare il flessore dell'anca sinistra.", "duration": 20}
            ],
            7: [  # sollevamento polpacci
                {"name": "posizione eretta", "description": "stai in piedi con i piedi alla larghezza dei fianchi vicino al muro per supporto.", "duration": 2},
                {"name": "alzati sulle punte", "description": "solleva i talloni da terra alzandoti sulle punte dei piedi.", "duration": 3},
                {"name": "mantieni posizione", "description": "mantieni la posizione alzata mantenendo l'equilibrio.", "duration": 2},
                {"name": "abbassa lentamente", "description": "abbassa lentamente i talloni a terra.", "duration": 3}
            ],
            8: [  # cerchi con le braccia
                {"name": "braccia estese", "description": "stai in piedi con le braccia estese parallele al pavimento.", "duration": 2},
                {"name": "piccoli cerchi avanti", "description": "fai piccoli movimenti circolari in avanti con entrambe le braccia.", "duration": 10},
                {"name": "piccoli cerchi indietro", "description": "fai piccoli movimenti circolari all'indietro con entrambe le braccia.", "duration": 10},
                {"name": "posizione di riposo", "description": "abbassa le braccia ai lati e rilassati.", "duration": 3}
            ],
            9: [  # rinforzo quadricipiti
                {"name": "seduto pronto", "description": "siediti sulla sedia con la schiena dritta e i piedi appoggiati.", "duration": 3},
                {"name": "estendi gamba destra", "description": "raddrizza il ginocchio destro sollevando il piede da terra.", "duration": 5},
                {"name": "mantieni estensione", "description": "mantieni la gamba dritta e contrai il muscolo della coscia.", "duration": 3},
                {"name": "abbassa gamba destra", "description": "abbassa lentamente il piede destro a terra.", "duration": 3},
                {"name": "estendi gamba sinistra", "description": "raddrizza il ginocchio sinistro sollevando il piede da terra.", "duration": 5}
            ],
            10: [  # rotazioni collo
                {"name": "posizione neutra", "description": "siediti o stai in piedi con la testa in posizione neutra.", "duration": 2},
                {"name": "gira a destra", "description": "gira lentamente la testa per guardare sopra la spalla destra.", "duration": 5},
                {"name": "ritorna al centro", "description": "ritorna lentamente la testa al centro.", "duration": 2},
                {"name": "gira a sinistra", "description": "gira lentamente la testa per guardare sopra la spalla sinistra.", "duration": 5}
            ],
            11: [  # equilibrio su una gamba
                {"name": "posizione di partenza", "description": "stai in piedi vicino al muro con i piedi insieme per supporto.", "duration": 3},
                {"name": "solleva gamba destra", "description": "solleva leggermente il piede destro da terra mantenendo l'equilibrio sul sinistro.", "duration": 30},
                {"name": "abbassa gamba destra", "description": "rimetti il piede destro a terra.", "duration": 2},
                {"name": "solleva gamba sinistra", "description": "solleva leggermente il piede sinistro da terra mantenendo l'equilibrio sul destro.", "duration": 30}
            ],
            12: [  # flessione polso
                {"name": "braccio esteso", "description": "estendi il braccio destro in avanti con il palmo rivolto verso il basso.", "duration": 2},
                {"name": "fletti polso in basso", "description": "piega il polso verso il basso puntando le dita verso il pavimento.", "duration": 10},
                {"name": "fletti polso in alto", "description": "piega il polso verso l'alto puntando le dita verso il soffitto.", "duration": 10},
                {"name": "cambia braccio", "description": "ripeti la sequenza con il braccio sinistro.", "duration": 22}
            ],
            13: [  # ponte glutei
                {"name": "posizione supina", "description": "sdraiati sulla schiena con le ginocchia piegate e i piedi appoggiati.", "duration": 3},
                {"name": "solleva fianchi", "description": "stringi i glutei e solleva i fianchi creando una linea retta.", "duration": 5},
                {"name": "mantieni ponte", "description": "mantieni la posizione respirando normalmente.", "duration": 3},
                {"name": "abbassa fianchi", "description": "abbassa lentamente i fianchi alla posizione iniziale.", "duration": 3}
            ],
            14: [  # pompe caviglie
                {"name": "posizione seduta", "description": "siediti con le gambe estese o sdraiati comodamente.", "duration": 2},
                {"name": "punta piedi", "description": "punta entrambi i piedi lontano dal corpo.", "duration": 2},
                {"name": "fletti piedi", "description": "tira entrambi i piedi verso il corpo.", "duration": 2},
                {"name": "continua movimento", "description": "alterna puntare e flettere i piedi ritmicamente.", "duration": 4}
            ],
            15: [  # flessioni al muro
                {"name": "posizione al muro", "description": "stai a distanza di un braccio dal muro con i palmi appoggiati.", "duration": 3},
                {"name": "piegati in avanti", "description": "inclina lentamente il corpo verso il muro piegando i gomiti.", "duration": 3},
                {"name": "spingi indietro", "description": "spingi il corpo alla posizione iniziale.", "duration": 3},
                {"name": "ripristina posizione", "description": "assicurati della forma corretta prima della prossima ripetizione.", "duration": 2}
            ]
        }

        with self.driver.session() as session:
            step_id = 1
            for exercise_id, steps in exercise_steps.items():
                for order, step in enumerate(steps, 1):
                    session.run(
                        "CREATE (s:ExerciseStep {name: $name, description: $description, "
                        "duration: $duration, id: $id})",
                        name=step["name"],
                        description=step["description"],
                        duration=step["duration"],
                        id=step_id
                    )
                    
                    # Create has_step relationship
                    session.run(
                        "MATCH (e:Exercise {id: $exercise_id}), (s:ExerciseStep {id: $step_id}) "
                        "CREATE (e)-[:has_step {order: $order}]->(s)",
                        exercise_id=exercise_id,
                        step_id=step_id,
                        order=order
                    )
                    step_id += 1

    def create_angles(self):
        angles = [
            {"limb1": "braccio_superiore", "limb2": "avambraccio", "angleValue": 90},  # flessione gomito
            {"limb1": "avambraccio", "limb2": "mano", "angleValue": 180},  # estensione polso
            {"limb1": "coscia", "limb2": "tibia", "angleValue": 90},  # flessione ginocchio
            {"limb1": "tibia", "limb2": "piede", "angleValue": 90},  # dorsiflessione caviglia
            {"limb1": "busto", "limb2": "braccio_superiore", "angleValue": 45},  # abduzione spalla
            {"limb1": "busto", "limb2": "collo", "angleValue": 180},  # estensione collo
            {"limb1": "busto", "limb2": "coscia", "angleValue": 90},  # flessione anca
            {"limb1": "tibia", "limb2": "piede", "angleValue": 110},  # plantarflessione caviglia
            {"limb1": "braccio_superiore", "limb2": "busto", "angleValue": 180},  # estensione spalla
            {"limb1": "mano", "limb2": "avambraccio", "angleValue": 170}  # flessione polso
        ]

        with self.driver.session() as session:
            for i, angle in enumerate(angles):
                session.run(
                    "CREATE (a:Angle {limb1: $limb1, limb2: $limb2, "
                    "angleValue: $angleValue, id: $id})",
                    limb1=angle["limb1"],
                    limb2=angle["limb2"],
                    angleValue=angle["angleValue"],
                    id=i+1
                )

    def create_patients(self):
        patients = [
            {"name": "mario", "heart_rate": 85, "oxygenation_level": 95, "respiratory_rate": 18, "condition": "obesità e diabete"},
            {"name": "giuseppe", "heart_rate": 78, "oxygenation_level": 97, "respiratory_rate": 16, "condition": "diabete tipo 2"},
            {"name": "francesca", "heart_rate": 82, "oxygenation_level": 96, "respiratory_rate": 17, "condition": "trauma spalla"},
            {"name": "antonio", "heart_rate": 88, "oxygenation_level": 94, "respiratory_rate": 19, "condition": "recupero chirurgia ginocchio"},
            {"name": "giulia", "heart_rate": 75, "oxygenation_level": 98, "respiratory_rate": 15, "condition": "lesione spinale"}
        ]

        with self.driver.session() as session:
            for i, patient in enumerate(patients):
                session.run(
                    "CREATE (p:Patient {name: $name, heart_rate: $heart_rate, "
                    "oxygenation_level: $oxygenation_level, respiratory_rate: $respiratory_rate, "
                    "condition: $condition, id: $id})",
                    name=patient["name"],
                    heart_rate=patient["heart_rate"],
                    oxygenation_level=patient["oxygenation_level"],
                    respiratory_rate=patient["respiratory_rate"],
                    condition=patient["condition"],
                    id=i+1
                )

    def create_angle_relationships(self):
        # Create has_target_angle relationships between steps and angles
        step_angle_mapping = [
            (2, 1), (4, 1), (9, 1), (11, 1),  # Shoulder-elbow angle
            (5, 2), (7, 2),  # Elbow-wrist angle
            (8, 3), (10, 3), (22, 3), (24, 3),  # Hip-knee angle
            (26, 4), (29, 4), (31, 4),  # Knee-ankle angle
            (13, 5), (15, 5), (30, 5), (32, 5),  # Shoulder-arm angle
            (36, 6), (38, 6),  # Spine-neck angle
            (19, 7), (48, 7), (50, 7),  # Hip-back angle
            (27, 8), (53, 8), (55, 8),  # Ankle-foot angle
            (12, 9), (14, 9), (58, 9),  # Shoulder-back angle
            (44, 10), (46, 10)  # Wrist-hand angle
        ]

        with self.driver.session() as session:
            for step_id, angle_id in step_angle_mapping:
                session.run(
                    "MATCH (s:ExerciseStep {id: $step_id}), (a:Angle {id: $angle_id}) "
                    "CREATE (s)-[:has_target_angle]->(a)",
                    step_id=step_id,
                    angle_id=angle_id
                )

    def create_patient_exercise_relationships(self):
        # Assign exercises to patients with different week days
        patient_exercises = [
            (1, 1, 1), (1, 3, 3), (1, 7, 5),  # Mario: exercises for obesity/diabetes
            (2, 2, 2), (2, 9, 4), (2, 14, 6),  # Giuseppe: diabetes-focused
            (3, 3, 1), (3, 8, 3), (3, 15, 5),  # Francesca: shoulder trauma
            (4, 2, 2), (4, 9, 4), (4, 13, 6),  # Antonio: knee surgery
            (5, 5, 1), (5, 10, 3), (5, 13, 5)   # Giulia: spinal injury
        ]

        with self.driver.session() as session:
            for patient_id, exercise_id, week_day in patient_exercises:
                session.run(
                    "MATCH (p:Patient {id: $patient_id}), (e:Exercise {id: $exercise_id}) "
                    "CREATE (p)-[:has_exercise {week_day: $week_day}]->(e)",
                    patient_id=patient_id,
                    exercise_id=exercise_id,
                    week_day=week_day
                )

    def populate_database(self):
        print("Clearing existing data...")
        self.clear_database()
        
        print("Creating exercises...")
        self.create_exercises()
        
        print("Creating exercise steps...")
        self.create_exercise_steps()
        
        print("Creating angles...")
        self.create_angles()
        
        print("Creating patients...")
        self.create_patients()
        
        print("Creating angle relationships...")
        self.create_angle_relationships()
        
        print("Creating patient-exercise relationships...")
        self.create_patient_exercise_relationships()
        
        print("Database population completed!")

def main():
    # Database connection parameters
    URI = "bolt://localhost:7687"
    USER = "neo4j"
    PASSWORD = "password"  # Change this to your Neo4j password
    
    db = RehabilitationDB(URI, USER, PASSWORD)
    
    try:
        db.populate_database()
    finally:
        db.close()

if __name__ == "__main__":
    main()