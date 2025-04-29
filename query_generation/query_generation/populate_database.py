from neo4j import GraphDatabase
import os
from dotenv import load_dotenv

# Load environment variables from .env file
BASE_DIR = "/home/kimary/unipa/src/unipa_inner_speech"
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path)

uri = os.getenv("NEO4J_URI")
username = os.getenv("NEO4J_USERNAME")
password = os.getenv("NEO4J_PASSWORD")

# Data to populate the database
USERS = [
    {"name": "Alice", "age": 30, "gender": "female", "fabbisogno": {"calories": 2000, "proteins": 70, "carbs": 250, "fats": 60}, "allergies": ["gluten", "nuts"]},
    {"name": "Bob", "age": 25, "gender": "male", "fabbisogno": {"calories": 2500, "proteins": 80, "carbs": 300, "fats": 80}, "allergies": ["lactose"]},
    {"name": "Charlie", "age": 35, "gender": "male", "fabbisogno": {"calories": 1800, "proteins": 60, "carbs": 200, "fats": 50}, "allergies": ["eggs"]},
    {"name": "Diana", "age": 28, "gender": "female", "fabbisogno": {"calories": 2200, "proteins": 75, "carbs": 270, "fats": 70}, "allergies": ["peanuts"]},
    {"name": "Eve", "age": 22, "gender": "female", "fabbisogno": {"calories": 1900, "proteins": 65, "carbs": 240, "fats": 55}, "allergies": []},
]

RECIPES = [
    {"name": "pasta_al_pesto", "type": "primo", "calories": 450, "proteins": 12, "carbs": 85, "fats": 5, "allergens": ["gluten", "nuts"]},
    {"name": "risotto_ai_funghi", "type": "primo", "calories": 380, "proteins": 8, "carbs": 70, "fats": 4, "allergens": []},
    {"name": "branzino_al_forno", "type": "secondo", "calories": 250, "proteins": 30, "carbs": 0, "fats": 8, "allergens": []},
    {"name": "frittata_di_zucchine", "type": "secondo", "calories": 280, "proteins": 15, "carbs": 5, "fats": 12, "allergens": ["eggs"]},
    {"name": "panna_cotta", "type": "dolce", "calories": 400, "proteins": 6, "carbs": 35, "fats": 25, "allergens": ["lactose"]},
    {"name": "minestrone_di_verdure", "type": "primo", "calories": 300, "proteins": 10, "carbs": 60, "fats": 5, "allergens": []},
    {"name": "pollo_alla_griglia", "type": "secondo", "calories": 320, "proteins": 35, "carbs": 0, "fats": 8, "allergens": []},
    {"name": "vellutata_di_zucca", "type": "primo", "calories": 200, "proteins": 4, "carbs": 40, "fats": 6, "allergens": []},
    {"name": "tiramisu", "type": "dolce", "calories": 450, "proteins": 8, "carbs": 50, "fats": 20, "allergens": ["lactose", "eggs"]},
    {"name": "bistecca_alla_fiorentina", "type": "secondo", "calories": 600, "proteins": 50, "carbs": 0, "fats": 30, "allergens": []},
    {"name": "sorbetto_al_limone", "type": "dolce", "calories": 150, "proteins": 1, "carbs": 35, "fats": 0, "allergens": []},
    {"name": "frutta_fresca", "type": "dessert", "calories": 120, "proteins": 1, "carbs": 30, "fats": 0, "allergens": []},
    {"name": "quinoa_con_verdure", "type": "piatto_unico", "calories": 400, "proteins": 15, "carbs": 70, "fats": 10, "allergens": []},
    {"name": "pizza_margherita", "type": "primo", "calories": 600, "proteins": 20, "carbs": 90, "fats": 15, "allergens": ["gluten", "lactose"]},
    {"name": "lasagna_classica", "type": "primo", "calories": 700, "proteins": 25, "carbs": 80, "fats": 30, "allergens": ["gluten", "lactose"]},
    {"name": "insalata_greca", "type": "contorno", "calories": 250, "proteins": 6, "carbs": 10, "fats": 20, "allergens": ["lactose"]},
    {"name": "carbonara", "type": "primo", "calories": 500, "proteins": 20, "carbs": 60, "fats": 20, "allergens": ["gluten", "eggs", "lactose"]},
    {"name": "filetto_di_manzo", "type": "secondo", "calories": 400, "proteins": 45, "carbs": 0, "fats": 15, "allergens": []},
    {"name": "parmigiana_di_melanzane", "type": "piatto_unico", "calories": 450, "proteins": 18, "carbs": 40, "fats": 25, "allergens": ["lactose"]},
    {"name": "crema_di_zuppa_di_pomodoro", "type": "primo", "calories": 200, "proteins": 3, "carbs": 25, "fats": 5, "allergens": []},
    {"name": "torta_alle_mele", "type": "dolce", "calories": 300, "proteins": 4, "carbs": 60, "fats": 10, "allergens": ["gluten", "eggs"]},
    {"name": "calzone_farcito", "type": "primo", "calories": 550, "proteins": 22, "carbs": 70, "fats": 15, "allergens": ["gluten", "lactose"]},
    {"name": "zucchine_grigliate", "type": "contorno", "calories": 120, "proteins": 2, "carbs": 10, "fats": 5, "allergens": []},
    {"name": "pollo_al_curry", "type": "secondo", "calories": 450, "proteins": 40, "carbs": 5, "fats": 15, "allergens": []},
    {"name": "insalata_di_riso", "type": "primo", "calories": 320, "proteins": 10, "carbs": 60, "fats": 5, "allergens": []},
    {"name": "pane_con_burro", "type": "contorno", "calories": 300, "proteins": 7, "carbs": 50, "fats": 12, "allergens": ["gluten", "lactose"]},
    {"name": "crostini_vegetali", "type": "antipasto", "calories": 250, "proteins": 5, "carbs": 40, "fats": 8, "allergens": ["gluten"]},
    {"name": "spezzatino_con_patate", "type": "secondo", "calories": 500, "proteins": 35, "carbs": 30, "fats": 20, "allergens": []},
    {"name": "zuppa_di_fagioli", "type": "primo", "calories": 350, "proteins": 15, "carbs": 50, "fats": 10, "allergens": []},
]


# Function to populate Neo4j
def populate_neo4j(uri, username, password):
    driver = GraphDatabase.driver(uri, auth=(username, password))

    with driver.session() as session:
        # Clear existing data
        session.run("MATCH (n) DETACH DELETE n")
        print("Database cleared.")

        # Create users and their fabbisogno
        for user in USERS:
            session.run("""
                CREATE (u:User {name: $name, age: $age, gender: $gender})
                CREATE (f:Fabbisogno {calories: $calories, proteins: $proteins, carbs: $carbs, fats: $fats})
                CREATE (u)-[:HAS_FABBISOGNO]->(f)
            """, {
                "name": user["name"],
                "age": user["age"],
                "gender": user["gender"],
                "calories": user["fabbisogno"]["calories"],
                "proteins": user["fabbisogno"]["proteins"],
                "carbs": user["fabbisogno"]["carbs"],
                "fats": user["fabbisogno"]["fats"]
            })

            # Create relationships for allergies
            for allergen in user["allergies"]:
                session.run("""
                   MERGE (a:Allergen {name: $allergen})
                    WITH a
                    MATCH (u:User {name: $name})
                    CREATE (u)-[:IS_ALLERGIC_TO]->(a)
                """, {"allergen": allergen, "name": user["name"]})

        print("Users and fabbisogno populated.")

        # Create recipes
        for recipe in RECIPES:
            recipe["name"] = recipe["name"].replace("_", " ")
            session.run("""
                CREATE (r:Recipe {name: $name, type: $type, calories: $calories, proteins: $proteins, carbs: $carbs, fats: $fats})
            """, {
                "name": recipe["name"],
                "type": recipe["type"],
                "calories": recipe["calories"],
                "proteins": recipe["proteins"],
                "carbs": recipe["carbs"],
                "fats": recipe["fats"]
            })

            # Create relationships for allergens
            for allergen in recipe["allergens"]:
                session.run("""
                    MERGE (a:Allergen {name: $allergen})
                    WITH a
                    MATCH (r:Recipe {name: $name})
                    CREATE (r)-[:HAS_ALLERGEN]->(a)
                """, {"allergen": allergen, "name": recipe["name"]})

        print("Recipes and allergens populated.")

    driver.close()
    print("Database population completed.")

# Run the function
populate_neo4j(uri, username, password)
