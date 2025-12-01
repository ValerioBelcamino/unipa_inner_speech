from neo4j import GraphDatabase
from langchain_neo4j import Neo4jGraph

class RehabilitationDB:
    def __init__(self, uri, user, password):
        self.driver = GraphDatabase.driver(uri, auth=(user, password))

    def close(self):
        self.driver.close()

    def clear_database(self):
        with self.driver.session() as session:
            session.run("MATCH (n) DETACH DELETE n")

    def create_exercises(self):
        exercises = [
            {"name": "Alternating Arm Raises", "num_repetitions": 15, "balance": False, "target_joint": "shoulder"},
            {"name": "Lower Limb Stretching", "num_repetitions": 8, "balance": True, "target_joint": "knee"},
            {"name": "Shoulder Contraction", "num_repetitions": 12, "balance": False, "target_joint": "shoulder"},
            {"name": "Heel-Toe Walking", "num_repetitions": 20, "balance": True, "target_joint": "ankle"},
            {"name": "Seated Spinal Twist", "num_repetitions": 10, "balance": False, "target_joint": "spine"},
            {"name": "Hip Flexor Stretch", "num_repetitions": 6, "balance": True, "target_joint": "hip"},
            {"name": "Calf Raises", "num_repetitions": 15, "balance": True, "target_joint": "ankle"},
            {"name": "Arm Circles", "num_repetitions": 12, "balance": False, "target_joint": "shoulder"},
            {"name": "Quadriceps Strengthening", "num_repetitions": 10, "balance": False, "target_joint": "knee"},
            {"name": "Neck Rotations", "num_repetitions": 8, "balance": False, "target_joint": "neck"},
            {"name": "Single Leg Balance", "num_repetitions": 5, "balance": True, "target_joint": "ankle"},
            {"name": "Wrist Flexion", "num_repetitions": 15, "balance": False, "target_joint": "wrist"},
            {"name": "Glute Bridge", "num_repetitions": 12, "balance": False, "target_joint": "hip"},
            {"name": "Ankle Pumps", "num_repetitions": 20, "balance": False, "target_joint": "ankle"},
            {"name": "Wall Push-Ups", "num_repetitions": 10, "balance": False, "target_joint": "shoulder"},
            {"name": "A", "num_repetitions": 12, "balance": False, "target_joint": "shoulder"},
            {"name": "B", "num_repetitions": 12, "balance": False, "target_joint": "shoulder"}
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
            1: [  # Alternating Arm Raises
                {"name": "Initial Position", "description": "Stand with feet shoulder-width apart, arms at your sides.", "duration": 2},
                {"name": "Raise Right Arm", "description": "Lift the right arm forward to shoulder height with a straight elbow.", "duration": 3},
                {"name": "Lower Right Arm", "description": "Slowly lower the right arm back to the initial position.", "duration": 2},
                {"name": "Raise Left Arm", "description": "Lift the left arm forward to shoulder height with a straight elbow.", "duration": 3},
                {"name": "Lower Left Arm", "description": "Slowly lower the left arm back to the initial position.", "duration": 2}
            ],
            2: [  # Lower Limb Stretching
                {"name": "Seated Position", "description": "Sit at the edge of the chair with a straight back and feet flat.", "duration": 3},
                {"name": "Extend Right Leg", "description": "Straighten the right leg and flex the foot upwards.", "duration": 15},
                {"name": "Return Right Leg", "description": "Slowly lower the right leg back to the initial position.", "duration": 3},
                {"name": "Extend Left Leg", "description": "Straighten the left leg and flex the foot upwards.", "duration": 15}
            ],
            3: [  # Shoulder Contraction
                {"name": "Standing Position", "description": "Stand with arms relaxed at your sides.", "duration": 2},
                {"name": "Squeeze Shoulder Blades", "description": "Squeeze the shoulder blades together while keeping arms straight.", "duration": 5},
                {"name": "Hold Position", "description": "Hold the contraction while breathing normally.", "duration": 3},
                {"name": "Release", "description": "Slowly release the shoulder blades back to the initial position.", "duration": 2}
            ],
            4: [  # Heel-Toe Walking
                {"name": "Starting Position", "description": "Stand with the right foot directly in front of the left.", "duration": 3},
                {"name": "Step Forward", "description": "Place the left heel directly in front of the right toe.", "duration": 2},
                {"name": "Continue Walking", "description": "Repeat heel-toe steps while maintaining balance.", "duration": 2}
            ],
            5: [  # Seated Spinal Twist
                {"name": "Seated Position", "description": "Sit up straight with feet flat and hands on shoulders.", "duration": 3},
                {"name": "Twist to the Right", "description": "Twist the torso to the right while keeping hips facing forward.", "duration": 5},
                {"name": "Return to Center", "description": "Slowly return to the initial position.", "duration": 2},
                {"name": "Twist to the Left", "description": "Twist the torso to the left while keeping hips facing forward.", "duration": 5}
            ],
            6: [  # Hip Flexor Stretch
                {"name": "Lunge Position", "description": "Step forward with the right foot into a lunge position.", "duration": 3},
                {"name": "Lower Hips", "description": "Gently lower the hips while keeping the knee in front of the ankle.", "duration": 20},
                {"name": "Change Leg", "description": "Switch to the position with the left foot forward.", "duration": 3},
                {"name": "Left Stretch", "description": "Lower the hips to stretch the left hip flexor.", "duration": 20}
            ],
            7: [  # Calf Raises
                {"name": "Standing Position", "description": "Stand with feet hip-width apart near a wall for support.", "duration": 2},
                {"name": "Rise on Toes", "description": "Lift heels off the ground, rising onto the toes.", "duration": 3},
                {"name": "Hold Position", "description": "Maintain the raised position while balancing.", "duration": 2},
                {"name": "Lower Slowly", "description": "Slowly lower the heels back to the ground.", "duration": 3}
            ],
            8: [  # Arm Circles
                {"name": "Arms Extended", "description": "Stand with arms extended parallel to the ground.", "duration": 2},
                {"name": "Small Circles Forward", "description": "Make small circular movements forward with both arms.", "duration": 10},
                {"name": "Small Circles Backward", "description": "Make small circular movements backward with both arms.", "duration": 10},
                {"name": "Rest Position", "description": "Lower arms to the sides and relax.", "duration": 3}
            ],
            9: [  # Quadriceps Strengthening
                {"name": "Ready Position", "description": "Sit on the chair with a straight back and feet flat.", "duration": 3},
                {"name": "Extend Right Leg", "description": "Straighten the right knee by lifting the foot off the ground.", "duration": 5},
                {"name": "Hold Extension", "description": "Keep the leg straight and contract the thigh muscle.", "duration": 3},
                {"name": "Lower Right Leg", "description": "Slowly lower the right foot to the ground.", "duration": 3},
                {"name": "Extend Left Leg", "description": "Straighten the left knee by lifting the foot off the ground.", "duration": 5}
            ],
            10: [  # Neck Rotations
                {"name": "Neutral Position", "description": "Sit or stand with the head in a neutral position.", "duration": 2},
                {"name": "Turn to the Right", "description": "Slowly turn the head to look over the right shoulder.", "duration": 5},
                {"name": "Return to Center", "description": "Slowly return the head to the center.", "duration": 2},
                {"name": "Turn to the Left", "description": "Slowly turn the head to look over the left shoulder.", "duration": 5}
            ],
            11: [  # Single Leg Balance
                {"name": "Starting Position", "description": "Stand near a wall with feet together for support.", "duration": 3},
                {"name": "Lift Right Leg", "description": "Slightly lift the right foot off the ground while balancing on the left.", "duration": 30},
                {"name": "Lower Right Leg", "description": "Place the right foot back on the ground.", "duration": 2},
                {"name": "Lift Left Leg", "description": "Slightly lift the left foot off the ground while balancing on the right.", "duration": 30}
            ],
            12: [  # Wrist Flexion
                {"name": "Arm Extended", "description": "Extend the right arm forward with the palm facing down.", "duration": 2},
                {"name": "Flex Wrist Down", "description": "Bend the wrist down pointing fingers towards the floor.", "duration": 10},
                {"name": "Flex Wrist Up", "description": "Bend the wrist up pointing fingers towards the ceiling.", "duration": 10},
                {"name": "Change Arm", "description": "Repeat the sequence with the left arm.", "duration": 22}
            ],
            13: [  # Glute Bridge
                {"name": "Supine Position", "description": "Lie on your back with knees bent and feet flat.", "duration": 3},
                {"name": "Lift Hips", "description": "Squeeze the glutes and lift the hips to create a straight line.", "duration": 5},
                {"name": "Hold Bridge", "description": "Maintain the position while breathing normally.", "duration": 3},
                {"name": "Lower Hips", "description": "Slowly lower the hips back to the initial position.", "duration": 3}
            ],
            14: [  # Ankle Pumps
                {"name": "Seated Position", "description": "Sit with legs extended or lie comfortably.", "duration": 2},
                {"name": "Point Toes", "description": "Point both feet away from the body.", "duration": 2},
                {"name": "Flex Feet", "description": "Pull both feet towards the body.", "duration": 2},
                {"name": "Continue Movement", "description": "Alternate pointing and flexing feet rhythmically.", "duration": 4}
            ],
            15: [  # Wall Push-Ups
                {"name": "Position at Wall", "description": "Stand an arm's length from the wall with palms pressed.", "duration": 3},
                {"name": "Lean Forward", "description": "Slowly lean your body towards the wall by bending your elbows.", "duration": 3},
                {"name": "Push Back", "description": "Push your body back to the initial position.", "duration": 3},
                {"name": "Restore Position", "description": "Ensure correct form before the next repetition.", "duration": 2}
            ],
            16: [  # Shoulder Flexion A
                {"name": "Initial Position", "description": "Stand with feet shoulder-width apart, arms at your sides.", "duration": 2},
                {"name": "Raise Right Arm", "description": "Slowly lift the right arm forward to 180 degrees above the head.", "duration": 4},
                {"name": "Hold Position", "description": "Keep the arm extended above the head with a straight elbow.", "duration": 3},
                {"name": "Lower Right Arm", "description": "Slowly lower the right arm back to the initial position.", "duration": 3},
                {"name": "Raise Left Arm", "description": "Slowly lift the left arm forward to 180 degrees above the head.", "duration": 4},
                {"name": "Lower Left Arm", "description": "Slowly lower the left arm back to the initial position.", "duration": 3}
            ],
            17: [  # Shoulder Adduction B
                {"name": "Initial Position", "description": "Stand with arms extended laterally at shoulder height.", "duration": 2},
                {"name": "Bring Arms Towards Body", "description": "Move both arms towards the center of the body while keeping them straight.", "duration": 4},
                {"name": "Cross Arms", "description": "Cross the arms in front of the chest reaching 180 degrees of adduction.", "duration": 3},
                {"name": "Hold Position", "description": "Maintain crossed arms while breathing normally.", "duration": 2},
                {"name": "Return Arms Laterally", "description": "Slowly return the arms to the initial lateral position.", "duration": 4},
                {"name": "Rest", "description": "Relax the arms at your sides.", "duration": 2}
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
            # Existing angles (targets)
            {"limb1": "upper_arm", "limb2": "forearm", "angleValue": 90},  # 1: elbow flexion
            {"limb1": "forearm", "limb2": "hand", "angleValue": 180},  # 2: wrist extension
            {"limb1": "thigh", "limb2": "shin", "angleValue": 90},  # 3: knee flexion
            {"limb1": "shin", "limb2": "foot", "angleValue": 90},  # 4: ankle dorsiflexion
            {"limb1": "torso", "limb2": "upper_arm", "angleValue": 90},  # 5: shoulder abduction
            {"limb1": "torso", "limb2": "neck", "angleValue": 0},  # 6: neck neutral
            {"limb1": "torso", "limb2": "thigh", "angleValue": 90},  # 7: hip flexion
            {"limb1": "shin", "limb2": "foot", "angleValue": 110},  # 8: ankle plantarflexion
            {"limb1": "upper_arm", "limb2": "torso", "angleValue": 180},  # 9: shoulder extension
            {"limb1": "hand", "limb2": "forearm", "angleValue": 170},  # 10: wrist flexion
            {"limb1": "torso", "limb2": "upper_arm", "angleValue": 180},  # 11: shoulder flexion vertical
            {"limb1": "upper_arm", "limb2": "torso", "angleValue": 0},  # 12: shoulder adduction
            
            # Initial/neutral positions
            {"limb1": "torso", "limb2": "upper_arm", "angleValue": 0},  # 13: arms at sides
            {"limb1": "upper_arm", "limb2": "forearm", "angleValue": 0},  # 14: elbow extended
            {"limb1": "thigh", "limb2": "shin", "angleValue": 0},  # 15: knee extended
            {"limb1": "shin", "limb2": "foot", "angleValue": 0},  # 16: ankle neutral
            {"limb1": "torso", "limb2": "neck", "angleValue": 0},  # 17: neck neutral
            {"limb1": "forearm", "limb2": "hand", "angleValue": 0},  # 18: wrist neutral
            {"limb1": "torso", "limb2": "upper_arm", "angleValue": 90},  # 19: arms at shoulder height
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
            # Exercise 1: Alternating Arm Raises
            (1, 13),  # Initial Position - arms at sides (0°)
            (2, 11),  # Raise Right Arm - shoulder flexion (180°)
            (3, 13),  # Lower Right Arm - return to initial (0°)
            (4, 11),  # Raise Left Arm - shoulder flexion (180°)
            (5, 13),  # Lower Left Arm - return to initial (0°)
            
            # Exercise 2: Lower Limb Stretching
            (6, 15),  # Seated Position - knee extended (0°)
            (7, 3),   # Extend Right Leg - knee flexion (90°)
            (8, 15),  # Return Right Leg - knee extended (0°)
            (9, 3),   # Extend Left Leg - knee flexion (90°)
            
            # Exercise 3: Shoulder Contraction
            (10, 13), # Standing Position - arms relaxed (0°)
            (11, 9),  # Squeeze Shoulder Blades - shoulder extension (180°)
            (12, 9),  # Hold Position - maintain shoulder extension (180°)
            (13, 13), # Release - return to relaxed position (0°)
            
            # Exercise 4: Heel-Toe Walking
            (14, 16), # Starting Position - ankle neutral (0°)
            (15, 4),  # Step Forward - ankle dorsiflexion (90°)
            (16, 16), # Continue Walking - return to neutral (0°)
            
            # Exercise 5: Seated Spinal Twist
            (17, 17), # Seated Position - torso neutral (0°)
            (18, 17), # Twist to the Right - maintain twist (0°)
            (19, 17), # Return to Center - torso neutral (0°)
            (20, 17), # Twist to the Left - maintain twist (0°)
            
            # Exercise 6: Hip Flexor Stretch
            (21, 7),  # Lunge Position - hip flexion (90°)
            (22, 7),  # Lower Hips - maintain hip flexion (90°)
            (23, 7),  # Change Leg - switch to left leg (90°)
            (24, 7),  # Left Stretch - maintain hip flexion (90°)
            
            # Exercise 7: Calf Raises
            (25, 16), # Standing Position - ankle neutral (0°)
            (26, 8),  # Rise on Toes - plantarflexion (110°)
            (27, 16), # Hold Position - maintain plantarflexion (110°)
            (28, 16), # Lower Slowly - return to neutral (0°)
            
            # Exercise 8: Arm Circles
            (29, 19), # Arms Extended - arms at shoulder height (90°)
            (30, 19), # Small Circles Forward - maintain (90°)
            (31, 19), # Small Circles Backward - maintain (90°)
            (32, 13), # Rest Position - arms at sides (0°)
            
            # Exercise 9: Quadriceps Strengthening
            (33, 3),  # Ready Position - knee extended (0°)
            (34, 15), # Extend Right Leg - knee flexion (90°)
            (35, 15), # Hold Extension - maintain (90°)
            (36, 3),  # Lower Right Leg - return to extended (0°)
            (37, 15), # Extend Left Leg - knee flexion (90°)
            
            # Exercise 10: Neck Rotations
            (38, 17), # Neutral Position - neck neutral (0°)
            (39, 6),  # Turn to the Right - maintain rotation (0°)
            (40, 17), # Return to Center - neck neutral (0°)
            (41, 6),  # Turn to the Left - maintain rotation (0°)
            
            # Exercise 11: Single Leg Balance
            (42, 16), # Starting Position - ankle neutral (0°)
            (43, 16), # Lift Right Leg - maintain balance (0°)
            (44, 16), # Lower Right Leg - return to neutral (0°)
            (45, 16), # Lift Left Leg - maintain balance (0°)
            
            # Exercise 12: Wrist Flexion
            (46, 18), # Arm Extended - wrist neutral (0°)
            (47, 10), # Flex Wrist Down - wrist flexion (90°)
            (48, 2),  # Flex Wrist Up - wrist extension (0°)
            (49, 18), # Change Arm - wrist neutral (0°)
            
            # Exercise 13: Glute Bridge
            (50, 3),  # Supine Position - knees bent (0°)
            (51, 7),  # Lift Hips - hip flexion (90°)
            (52, 7),  # Hold Bridge - maintain (90°)
            (53, 3),  # Lower Hips - return to initial (0°)
            
            # Exercise 14: Ankle Pumps
            (54, 16), # Seated Position - ankle neutral (0°)
            (55, 8),  # Point Toes - plantarflexion (110°)
            (56, 4),  # Flex Feet - dorsiflexion (90°)
            (57, 16), # Continue Movement - maintain neutral (0°)
            
            # Exercise 15: Wall Push-Ups
            (58, 14), # Position at Wall - elbows extended (0°)
            (59, 1),  # Lean Forward - elbows flexed (90°)
            (60, 14), # Push Back - return to initial (0°)
            (61, 14), # Restore Position - maintain (0°)
            
            # Exercise 16: Shoulder Flexion A
            (62, 13), # Initial Position - arms at sides (0°)
            (63, 11), # Raise Right Arm - shoulder flexion (180°)
            (64, 11), # Hold Position - maintain (180°)
            (65, 13), # Lower Right Arm - return to initial (0°)
            (66, 11), # Raise Left Arm - shoulder flexion (180°)
            (67, 13), # Lower Left Arm - return to initial (0°)
            
            # Exercise 17: Shoulder Adduction B
            (68, 19), # Initial Position - arms extended (90°)
            (69, 12), # Bring Arms Towards Body - maintain (0°)
            (70, 12), # Cross Arms - maintain (0°)
            (71, 12), # Hold Position - maintain (0°)
            (72, 19), # Return Arms Laterally - return to initial (90°)
            (73, 13), # Rest - arms at sides (0°)
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
            (5, 5, 1), (5, 10, 3), (5, 13, 5),  # Giulia: spinal injury
            (5, 16, 2), (5, 17, 4)  # Giulia: new shoulder exercises A and B
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
    # URI = "bolt://localhost:7687"
    URI = "neo4j+s://659becdb.databases.neo4j.io"
    USER = "neo4j"
    # PASSWORD = "password"  # Change this to your Neo4j password
    PASSWORD = "v4aOGzV9fEBAi8qKAYd8ZKHbS7vo81j3ej6vfFHELOw"
    
    db = RehabilitationDB(URI, USER, PASSWORD)
    
    try:
        db.populate_database()
    finally:
        db.close()

if __name__ == "__main__":
    main()