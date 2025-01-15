from neo4j import Record
import csv
import os

# Assuming your workspace structure is standard
ws_dir = os.getenv("ROS2_WORKSPACE", "/home/belca/Desktop/ros2_foxy_ws")  # Replace with your workspace path if needed
source_dir = os.path.join(ws_dir, 'src', 'clingo_nodes', 'clingo_nodes')

def generate_pl_file(fabbisogno_records, recipe_records):
    # File path for the output .pl file
    output_file = os.path.join(source_dir, "diet_data.pl")
    fabbisogno_record = fabbisogno_records[0]

    with open(output_file, "w") as f:
        # Write fabbisogno
        f.write("% Fatti: fabbisogno calorico e macronutrienti\n")
        f.write(f"fabbisogno(calorie, {fabbisogno_record['daily_calories']}).\n")
        f.write(f"fabbisogno(proteine, {fabbisogno_record['daily_proteins']}).\n")
        f.write(f"fabbisogno(carboidrati, {fabbisogno_record['daily_carbs']}).\n")
        f.write(f"fabbisogno(grassi, {fabbisogno_record['daily_fats']}).\n")
        f.write("\n")

        # Write recipes
        grouped_recipes = {}
        for record in recipe_records:
            grouped_recipes.setdefault(record["type"], []).append(record)

        for dish_type, dish_list in grouped_recipes.items():
            f.write(f"% {dish_type.capitalize()} Dishes\n")
            for dish in dish_list:
                f.write(
                    f"ricetta({dish['dish']}, {dish['type']}, {dish['calories']}, {dish['proteins']}, "
                    f"{dish['carbs']}, {dish['fats']}).\n"
                )
            f.write("\n")

    print(f"Prolog file '{output_file}' generated successfully.")


def generate_csv_file(fabbisogno_records, recipe_records):
    # Write CSV file
    output_file = os.path.join(source_dir, "diet_data.csv")
    with open(output_file, mode="w", newline="") as csvfile:
        fieldnames = ["dish", "type", "calories", "proteins", "carbs", "fats"]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        # Write header
        writer.writeheader()

        # Write recipe records
        for record in recipe_records:
            writer.writerow({
                "dish": record["dish"],
                "type": record["type"],
                "calories": record["calories"],
                "proteins": record["proteins"],
                "carbs": record["carbs"],
                "fats": record["fats"],
            })

    print(f"CSV file '{output_file}' generated successfully.")