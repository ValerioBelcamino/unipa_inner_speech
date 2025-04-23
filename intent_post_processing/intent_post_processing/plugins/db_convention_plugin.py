from unidecode import unidecode


def param_to_lower(intent_parameters: dict, action_id: int):
    # print(f"\033[95m[param_to_lower] parameters = {{'tool_output': {intent_parameters}, 'tool_id': {action_id}}}\033[0m")

    '''Updates the parameters extracted by the tools to match our KG conventions'''
    for k,v in intent_parameters.items():
        # AddToDatabase
        if action_id == '1':
            if k == 'nome_utente':
                # Cast to lowercase
                v = v.lower()
            elif k == 'intolleranze':
                for i in range(len(v)):
                    v[i] = v[i].lower()
                    # v[i] = v[i].replace(' ', '_')
        # DishInfo
        elif action_id == '2':
            if k == 'controllo_ingredienti':
                for i in range(len(v)):
                    v[i] = v[i].lower()
                    # v[i] = v[i].replace(' ', '_')
            else:
                # Cast to lowercase
                v = v.lower()
                # Replace spaces with underscores
                # v = v.replace(' ', '_')
            intent_parameters[k] = v
        # SubstituteDish
        elif action_id == '3':
            # Remove accents from italian names of the week
            if k == 'giorno':
                v = unidecode(v)

            if k in ['ingredienti_rimossi', 'ingredienti_preferiti', 'solo_questi_ingredienti']:
                for i in range(len(v)):
                    v[i] = v[i].lower()
                    # v[i] = v[i].replace(' ', '_')
            else:
                # Cast to lowercase
                v = v.lower()
                # Replace spaces with underscores
                # v = v.replace(' ', '_')
            intent_parameters[k] = v


def check_user_weekly_plan(intent_parameters: dict, action_id: int, db_driver):
    # print(f"\033[95m[check_user_weekly_plan] parameters = {{'person_name': {intent_parameters}, 'action_id': {action_id}, 'driver': {db_driver}}}\033[0m")

    # we only have to check the user weekly plan for the meal preparation
    if action_id == 3:

        query = """
        MATCH (p:Person {name: $name})-[:SHOULD_EAT]->(d:Dish)
        WITH p, collect(DISTINCT d) AS dishes
        MATCH (p)-[r:SHOULD_EAT]->(:Dish)
        WITH p, collect(DISTINCT r.day) AS plannedDays
        RETURN p.name AS person, plannedDays,
            size(plannedDays) AS daysCovered,
            CASE WHEN size(plannedDays) = 7 THEN true ELSE false END AS hasWeeklyPlan
        """
        
        with db_driver.session() as session:
            result = session.run(query, name=intent_parameters)
            record = result.single()  # Expecting one result
            if record:
                intent_parameters["hasWeeklyPlan"] = record["hasWeeklyPlan"]
    return intent_parameters