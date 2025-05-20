from neo4j import GraphDatabase
from dotenv import load_dotenv
import os

# Load environment variables from .env file
BASE_DIR = "/home/belca/Desktop/ros2_humble_ws/src"
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path, override=True)

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

# Sample diet plan for each user.
# Each key is a user name; its value is a list of days.
# For each day, a dictionary defines a 'day' (e.g., Monday) and a dictionary of meals.
DIET_PLANS = {
    "Alice": [
        {
            "day": "Monday",
            "meals": {
                "Breakfast": ["panna cotta", "frutta fresca"],
                "Lunch": ["pasta al pesto", "insalata greca"],
                "Dinner": ["branzino al forno", "zucchine grigliate"]
            }
        },
        {
            "day": "Tuesday",
            "meals": {
                "Breakfast": ["sorbetto al limone"],
                "Lunch": ["risotto ai funghi", "minestrone di verdure"],
                "Dinner": ["pollo alla griglia", "insalata di riso"]
            }
        }
    ],
    "Bob": [
        {
            "day": "Monday",
            "meals": {
                "Breakfast": ["pane con burro"],
                "Lunch": ["carbonara", "insalata greca"],
                "Dinner": ["filetto di manzo", "zucchine grigliate"]
            }
        },
        {
            "day": "Wednesday",
            "meals": {
                "Breakfast": ["tiramisu"],
                "Lunch": ["lasagna classica"],
                "Dinner": ["bistecca alla fiorentina", "crostini vegetali"]
            }
        }
    ]
    # You can add plans for other users as needed.
}


from neo4j import GraphDatabase
import uuid

def populate_neo4j(uri, username, password):
    driver = GraphDatabase.driver(uri, auth=(username, password))

    with driver.session() as session:
        # Clear existing data
        session.run("MATCH (n) DETACH DELETE n")
        print("Database cleared.")

        days_of_week = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]

        for user in USERS:
            user_name = user["name"]
            weekly_plan_name = f"{user_name} Weekly Plan"

            session.run("""
                CREATE (u:User {name: $name, age: $age, gender: $gender})
                CREATE (f:Fabbisogno {calories: $calories, proteins: $proteins, carbs: $carbs, fats: $fats})
                CREATE (wp:WeeklyPlan {name: $weekly_plan_name})
                CREATE (u)-[:HAS_FABBISOGNO]->(f)
                CREATE (u)-[:HAS_WEEKLY_PLAN]->(wp)
            """, {
                "name": user_name,
                "age": user["age"],
                "gender": user["gender"],
                "calories": user["fabbisogno"]["calories"],
                "proteins": user["fabbisogno"]["proteins"],
                "carbs": user["fabbisogno"]["carbs"],
                "fats": user["fabbisogno"]["fats"],
                "weekly_plan_name": weekly_plan_name
            })

            # Create relationships for allergies
            for allergen in user["allergies"]:
                session.run("""
                    MERGE (a:Allergen {name: $allergen})
                    WITH a
                    MATCH (u:User {name: $name})
                    CREATE (u)-[:IS_ALLERGIC_TO]->(a)
                """, {"allergen": allergen, "name": user_name})

            # Create 7 Daily Plans for this user
            for day in days_of_week:
                daily_plan_name = f"{day} {user_name}"
                session.run("""
                    MATCH (wp:WeeklyPlan {name: $weekly_plan_name})
                    CREATE (dp:DailyPlan {name: $daily_plan_name, day: $day})
                    CREATE (wp)-[:HAS_DAILY_PLAN]->(dp)
                """, {
                    "weekly_plan_name": weekly_plan_name,
                    "daily_plan_name": daily_plan_name,
                    "day": day
                })

        print("Users, dietary needs, and plans populated.")

        # Create recipes
        for recipe in RECIPES:
            recipe_name = recipe["name"].replace("_", " ")

            session.run("""
                CREATE (r:Recipe {name: $name, type: $type, calories: $calories, proteins: $proteins, carbs: $carbs, fats: $fats})
            """, {
                "name": recipe_name,
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
                    CREATE (r)-[:CONTAINS_ALLERGEN]->(a)
                """, {"allergen": allergen, "name": recipe_name})

        print("Recipes and allergens populated.")


        # Fill the daily plans with meals from DIET_PLANS
        for user_name, user_plans in DIET_PLANS.items():
            for plan in user_plans:
                day = plan["day"]
                meals = plan["meals"]

                # print(user_name)
                # print(day)
                # print((meals))
                # print('\n')

                # Fetch the Daily Plan for the specific user and day
                daily_plan_name = f"{day} {user_name}"

                # Add relationships for each meal
                for meal_type, meal_recipes in meals.items():
                    # Create a unique identifier for the meal node, using both the meal type and the day
                    meal_node_name = f"{meal_type} {day} {user_name}"

                    # Create a Meal node for this meal type (Breakfast, Lunch, Dinner)
                    session.run("""
                        MATCH (dp:DailyPlan {name: $daily_plan_name})
                        CREATE (m:Meal {name: $meal_node_name, type: $meal_type})
                        CREATE (dp)-[:HAS_MEAL]->(m)
                    """, {
                        "daily_plan_name": daily_plan_name,
                        "meal_node_name": meal_node_name,
                        "meal_type": meal_type
                    })

                    # For each recipe in this meal, create the relationship
                    for recipe_name in meal_recipes:
                        session.run("""
                            MATCH (m:Meal {name: $meal_node_name})
                            MATCH (r:Recipe {name: $recipe_name})
                            CREATE (m)-[:CONTAINS]->(r)
                        """, {
                            "meal_node_name": meal_node_name,
                            "recipe_name": recipe_name
                        })

        print("Daily plans filled with meals and recipes.")

     

    driver.close()

# Run the script
populate_neo4j(uri, username, password)
