from neo4j import GraphDatabase
import random
from dotenv import load_dotenv
import os

URI = "neo4j://localhost:7687"
USERNAME = "neo4j"
PASSWORD = "password"

# Connect to the database
driver = GraphDatabase.driver(URI, auth=(USERNAME, PASSWORD))

def create_database():
    with driver.session() as session:
        # Clear existing data
        session.run("MATCH (n) DETACH DELETE n")
        
        # Create allergens
        allergens = ["lattosio", "glutine", "frutta secca", "pesce", "crostacei", "soia", "uova"]
        for allergen in allergens:
            session.run(
                "CREATE (a:Allergen {name: $name})",
                name=allergen
            )
        
        # Create ingredients with nutrition values
        ingredients = [
            {"name": "farina", "carbs": 76, "proteins": 10, "fats": 1, "calories": 364, "allergens": ["glutine"]},
            {"name": "couscous", "carbs": 23, "proteins": 4, "fats": 0, "calories": 112, "allergens": ["glutine"]},
            {"name": "uova", "carbs": 1, "proteins": 13, "fats": 11, "calories": 155, "allergens": ["uova"]},
            {"name": "latte", "carbs": 5, "proteins": 3, "fats": 4, "calories": 64, "allergens": ["lattosio"]},
            {"name": "panna", "carbs": 3, "proteins": 2, "fats": 20, "calories": 193, "allergens": ["lattosio"]},
            {"name": "mascarpone", "carbs": 2, "proteins": 3, "fats": 40, "calories": 400, "allergens": ["lattosio"]},
            {"name": "pomodoro", "carbs": 4, "proteins": 1, "fats": 0, "calories": 18, "allergens": []},
            {"name": "mozzarella", "carbs": 2, "proteins": 22, "fats": 22, "calories": 280, "allergens": ["lattosio"]},
            {"name": "pasta", "carbs": 75, "proteins": 13, "fats": 2, "calories": 371, "allergens": ["glutine"]},
            {"name": "olio", "carbs": 0, "proteins": 0, "fats": 100, "calories": 884, "allergens": []},
            {"name": "parmigiano", "carbs": 4, "proteins": 33, "fats": 29, "calories": 431, "allergens": ["lattosio"]},
            {"name": "riso", "carbs": 80, "proteins": 7, "fats": 1, "calories": 362, "allergens": []},
            {"name": "tonno", "carbs": 0, "proteins": 25, "fats": 8, "calories": 184, "allergens": ["pesce"]},
            {"name": "mandorle", "carbs": 22, "proteins": 21, "fats": 49, "calories": 579, "allergens": ["frutta secca"]},
            {"name": "zucchero", "carbs": 100, "proteins": 0, "fats": 0, "calories": 394, "allergens": []},
            {"name": "funghi", "carbs": 3, "proteins": 3, "fats": 0, "calories": 22, "allergens": []},
            {"name": "gamberi", "carbs": 0, "proteins": 24, "fats": 1, "calories": 106, "allergens": ["crostacei"]},
            {"name": "cioccolato", "carbs": 61, "proteins": 5, "fats": 31, "calories": 546, "allergens": ["lattosio"]},
            {"name": "spinaci", "carbs": 4, "proteins": 3, "fats": 0, "calories": 23, "allergens": []},
            {"name": "ricotta", "carbs": 3, "proteins": 11, "fats": 10, "calories": 174, "allergens": ["lattosio"]},
            {"name": "limone", "carbs": 9, "proteins": 1, "fats": 0, "calories": 29, "allergens": []},
            {"name": "manzo", "carbs": 0, "proteins": 26, "fats": 15, "calories": 250, "allergens": []},
            {"name": "caffe", "carbs": 0, "proteins": 0, "fats": 0, "calories": 2, "allergens": []},
            {"name": "salmone", "carbs": 0, "proteins": 20, "fats": 13, "calories": 208, "allergens": ["pesce"]}
        ]
        
        for ing in ingredients:
            session.run(
                "CREATE (i:Ingredient {name: $name, carbs: $carbs, proteins: $proteins, fats: $fats, calories: $calories})",
                name=ing["name"], carbs=ing["carbs"], proteins=ing["proteins"], fats=ing["fats"], calories=ing["calories"]
            )
            
            # Connect ingredients to allergens
            for allergen in ing["allergens"]:
                session.run(
                    """
                    MATCH (i:Ingredient {name: $ing_name})
                    MATCH (a:Allergen {name: $allergen_name})
                    CREATE (i)-[:CONTAINS]->(a)
                    """,
                    ing_name=ing["name"], allergen_name=allergen
                )
        
        # Create dishes
        dishes = [
            {
                "name": "pasta al pomodoro", 
                "type": "primo", 
                "ingredients": ["pasta", "pomodoro", "olio", "parmigiano"],
                "carbs": 60, "proteins": 10, "fats": 15, "calories": 450
            },
            {
                "name": "risotto ai funghi", 
                "type": "primo", 
                "ingredients": ["riso", "funghi", "parmigiano", "olio"],
                "carbs": 65, "proteins": 8, "fats": 12, "calories": 420
            },
            {
                "name": "riso al pomodoro", 
                "type": "primo", 
                "ingredients": ["riso", "pomodoro", "olio"],
                "carbs": 60, "proteins": 7, "fats": 10, "calories": 380
            },
            {
                "name": "couscous", 
                "type": "primo", 
                "ingredients": ["couscous", "olio"],
                "carbs": 40, "proteins": 8, "fats": 8, "calories": 250
            },
            {
                "name": "lasagna", 
                "type": "primo", 
                "ingredients": ["pasta", "pomodoro", "mozzarella", "parmigiano"],
                "carbs": 50, "proteins": 18, "fats": 22, "calories": 420
            },
            {
                "name": "bistecca di manzo", 
                "type": "secondo", 
                "ingredients": ["manzo", "olio"],
                "carbs": 0, "proteins": 40, "fats": 25, "calories": 380
            },
            {
                "name": "insalata di tonno", 
                "type": "secondo", 
                "ingredients": ["tonno", "pomodoro", "olio"],
                "carbs": 5, "proteins": 30, "fats": 15, "calories": 300
            },
            {
                "name": "salmone alla griglia", 
                "type": "secondo", 
                "ingredients": ["salmone", "olio", "limone"],
                "carbs": 0, "proteins": 25, "fats": 15, "calories": 280
            },
            {
                "name": "tiramisu", 
                "type": "dolce", 
                "ingredients": ["uova", "zucchero", "caffe", "mascarpone"],
                "carbs": 45, "proteins": 8, "fats": 25, "calories": 420
            },
            {
                "name": "panna cotta", 
                "type": "dolce", 
                "ingredients": ["panna", "zucchero"],
                "carbs": 30, "proteins": 5, "fats": 20, "calories": 320
            },
            {
                "name": "pasta alla carbonara", 
                "type": "primo", 
                "ingredients": ["pasta", "uova", "parmigiano"],
                "carbs": 55, "proteins": 22, "fats": 18, "calories": 480
            },
            {
                "name": "cappuccino", 
                "type": "bevanda", 
                "ingredients": ["caffe", "latte"],
                "carbs": 5, "proteins": 3, "fats": 4, "calories": 70
            },
            {
                "name": "gamberi alla griglia", 
                "type": "secondo", 
                "ingredients": ["gamberi", "olio", "limone"],
                "carbs": 1, "proteins": 25, "fats": 10, "calories": 200
            },
            {
                "name": "ravioli di ricotta", 
                "type": "primo", 
                "ingredients": ["pasta", "ricotta", "spinaci"],
                "carbs": 50, "proteins": 15, "fats": 12, "calories": 380
            },
            {
                "name": "torta di mandorle", 
                "type": "dolce", 
                "ingredients": ["mandorle", "uova", "zucchero"],
                "carbs": 40, "proteins": 12, "fats": 30, "calories": 450
            },
            {
                "name": "frittata di spinaci",
                "type": "secondo",
                "ingredients": ["uova", "spinaci", "olio"],
                "carbs": 5, "proteins": 21, "fats": 20, "calories": 270
            }
        ]
        
        for dish in dishes:
            session.run(
                """
                CREATE (d:Dish {
                    name: $name, 
                    type: $type, 
                    carbs: $carbs, 
                    proteins: $proteins, 
                    fats: $fats, 
                    calories: $calories
                })
                """,
                name=dish["name"], type=dish["type"], 
                carbs=dish["carbs"], proteins=dish["proteins"], 
                fats=dish["fats"], calories=dish["calories"]
            )
            
            # Connect dishes to ingredients
            for ing_name in dish["ingredients"]:
                session.run(
                    """
                    MATCH (d:Dish {name: $dish_name})
                    MATCH (i:Ingredient {name: $ing_name})
                    CREATE (d)-[:CONTAINS]->(i)
                    """,
                    dish_name=dish["name"], ing_name=ing_name
                )
        
        # Create people
        people = [
            {
                "name": "marco", 
                "gender": "m", 
                "age": 35, 
                "carbs": 320, 
                "proteins": 120, 
                "fats": 90, 
                "calories": 2600, 
                "allergies": ["lattosio"]
            },
            {
                "name": "giulia", 
                "gender": "f", 
                "age": 28, 
                "carbs": 250, 
                "proteins": 90, 
                "fats": 60, 
                "calories": 1900, 
                "allergies": ["glutine", "frutta secca"]
            },
            {
                "name": "antonio", 
                "gender": "m", 
                "age": 42, 
                "carbs": 280, 
                "proteins": 140, 
                "fats": 80, 
                "calories": 2400, 
                "allergies": []
            },
            {
                "name": "sofia", 
                "gender": "f", 
                "age": 31, 
                "carbs": 220, 
                "proteins": 85, 
                "fats": 65, 
                "calories": 1800, 
                "allergies": ["pesce", "crostacei"]
            },
            {
                "name": "luca", 
                "gender": "m", 
                "age": 25, 
                "carbs": 350, 
                "proteins": 150, 
                "fats": 70, 
                "calories": 2700, 
                "allergies": ["uova"]
            },
            {
                "name": "anna", 
                "gender": "f", 
                "age": 26, 
                "carbs": 230, 
                "proteins": 100, 
                "fats": 70, 
                "calories": 1800, 
                "allergies": ["glutine", "lattosio"]
            },
            {
                "name": "valerio", 
                "gender": "m", 
                "age": 32, 
                "carbs": 280, 
                "proteins": 110, 
                "fats": 70, 
                "calories": 2400, 
                "allergies": []
            },
            {
                "name": "francesca", 
                "gender": "f", 
                "age": 29, 
                "carbs": 220, 
                "proteins": 90, 
                "fats": 65, 
                "calories": 2000, 
                "allergies": []
            },
            {
                "name": "giorgio", 
                "gender": "m", 
                "age": 30, 
                "carbs": 240, 
                "proteins": 95, 
                "fats": 70, 
                "calories": 2200, 
                "allergies": []
            },
            {
                "name": "alessandro", 
                "gender": "m", 
                "age": 27, 
                "carbs": 300, 
                "proteins": 120, 
                "fats": 80, 
                "calories": 2600, 
                "allergies": []
            }
        ]
        
        for person in people:
            session.run(
                """
                CREATE (p:Person {
                    name: $name, 
                    gender: $gender, 
                    age: $age, 
                    carbs: $carbs, 
                    proteins: $proteins, 
                    fats: $fats, 
                    calories: $calories
                })
                """,
                name=person["name"], gender=person["gender"], age=person["age"],
                carbs=person["carbs"], proteins=person["proteins"], 
                fats=person["fats"], calories=person["calories"]
            )
            
            # Connect people to allergens
            for allergen in person["allergies"]:
                session.run(
                    """
                    MATCH (p:Person {name: $person_name})
                    MATCH (a:Allergen {name: $allergen_name})
                    CREATE (p)-[:IS_ALLERGIC_TO]->(a)
                    """,
                    person_name=person["name"], allergen_name=allergen
                )
        
        # Create weekly meal plans
        days = ["lunedi", "martedi", "mercoledi", "giovedi", "venerdi", "sabato", "domenica"]
        meals = ["colazione", "pranzo", "cena"]
        
        # Dictionary to track which dishes contain allergens a person is allergic to
        incompatible_dishes = {}
        
        for person in people:
            # Find dishes that contain allergens this person is allergic to
            allergic_dishes = []
            for allergen in person["allergies"]:
                result = session.run(
                    """
                    MATCH (d:Dish)-[:CONTAINS]->(:Ingredient)-[:CONTAINS]->(:Allergen {name: $allergen_name})
                    RETURN d.name AS dish_name
                    """,
                    allergen_name=allergen
                )
                allergic_dishes.extend([record["dish_name"] for record in result])
            
            incompatible_dishes[person["name"]] = set(allergic_dishes)
            
        # Create meal plans for each person
        for person in people:
            # Get list of safe dishes for this person
            all_dishes = [dish["name"] for dish in dishes]
            safe_dishes = [d for d in all_dishes if d not in incompatible_dishes[person["name"]]]
            
            # Group dishes by type
            dish_types = {}
            for dish in dishes:
                if dish["name"] in safe_dishes:
                    if dish["type"] not in dish_types:
                        dish_types[dish["type"]] = []
                    dish_types[dish["type"]].append(dish["name"])
            
            # Create weekly meal plan
            for day in days:
                # Breakfast
                if "bevanda" in dish_types and dish_types["bevanda"]:
                    bevanda = random.choice(dish_types["bevanda"])
                    session.run(
                        """
                        MATCH (p:Person {name: $person_name})
                        MATCH (d:Dish {name: $dish_name})
                        CREATE (p)-[:SHOULD_EAT {day: $day, meal: $meal}]->(d)
                        """,
                        person_name=person["name"], dish_name=bevanda, day=day, meal="colazione"
                    )
                
                # Lunch - usually primo + secondo
                if "primo" in dish_types and dish_types["primo"]:
                    primo = random.choice(dish_types["primo"])
                    session.run(
                        """
                        MATCH (p:Person {name: $person_name})
                        MATCH (d:Dish {name: $dish_name})
                        CREATE (p)-[:SHOULD_EAT {day: $day, meal: $meal}]->(d)
                        """,
                        person_name=person["name"], dish_name=primo, day=day, meal="pranzo"
                    )
                
                if "secondo" in dish_types and dish_types["secondo"]:
                    secondo = random.choice(dish_types["secondo"])
                    session.run(
                        """
                        MATCH (p:Person {name: $person_name})
                        MATCH (d:Dish {name: $dish_name})
                        CREATE (p)-[:SHOULD_EAT {day: $day, meal: $meal}]->(d)
                        """,
                        person_name=person["name"], dish_name=secondo, day=day, meal="pranzo"
                    )
                
                # Dinner - primo or secondo + sometimes dolce on weekends
                if "primo" in dish_types and dish_types["primo"]:
                    if random.choice([True, False]):  # Sometimes primo, sometimes secondo
                        dish = random.choice(dish_types["primo"])
                        session.run(
                            """
                            MATCH (p:Person {name: $person_name})
                            MATCH (d:Dish {name: $dish_name})
                            CREATE (p)-[:SHOULD_EAT {day: $day, meal: $meal}]->(d)
                            """,
                            person_name=person["name"], dish_name=dish, day=day, meal="cena"
                        )
                    elif "secondo" in dish_types and dish_types["secondo"]:
                        dish = random.choice(dish_types["secondo"])
                        session.run(
                            """
                            MATCH (p:Person {name: $person_name})
                            MATCH (d:Dish {name: $dish_name})
                            CREATE (p)-[:SHOULD_EAT {day: $day, meal: $meal}]->(d)
                            """,
                            person_name=person["name"], dish_name=dish, day=day, meal="cena"
                        )
                
                # Add dolce for dinner on weekends
                if day in ["sabato", "domenica"] and "dolce" in dish_types and dish_types["dolce"]:
                    dolce = random.choice(dish_types["dolce"])
                    session.run(
                        """
                        MATCH (p:Person {name: $person_name})
                        MATCH (d:Dish {name: $dish_name})
                        CREATE (p)-[:SHOULD_EAT {day: $day, meal: $meal}]->(d)
                        """,
                        person_name=person["name"], dish_name=dolce, day=day, meal="cena"
                    )

if __name__ == "__main__":
    create_database()
    driver.close()
    print("Database successfully populated!")
