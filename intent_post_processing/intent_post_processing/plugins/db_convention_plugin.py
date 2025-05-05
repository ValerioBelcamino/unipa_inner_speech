from unidecode import unidecode


def param_to_lower(intent_parameters: dict, action_name: str):
    # print(f"\033[95m[param_to_lower] parameters = {{'tool_output': {intent_parameters}, 'action_name': {action_name}}}\033[0m")

    '''Updates the parameters extracted by the tools to match our KG conventions'''
    for k,v in intent_parameters.items():
        # AddToDatabase
        if action_name == 'AddToDatabase':
            if k == 'nome_utente':
                # Cast to lowercase
                v = v.lower()
            elif k == 'intolleranze':
                for i in range(len(v)):
                    v[i] = v[i].lower()
                    # v[i] = v[i].replace(' ', '_')
        # DishInfo
        elif action_name == 'DishInfo':
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
        elif action_name == 'SubstituteDish':
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


def check_user_weekly_plan(intent_parameters: dict, action_name: str, db_adapter):
    # print(f"\033[95m[check_user_weekly_plan] parameters = {{'person_name': {intent_parameters}, 'action_name': {action_name}, 'db_adapter': {db_adapter}}}\033[0m")

    # we only have to check the user weekly plan for the meal preparation
    if action_name == 'SubstituteDish':

        query = """
        MATCH (p:Person {name: $name})-[:SHOULD_EAT]->(d:Dish)
        WITH p, collect(DISTINCT d) AS dishes
        MATCH (p)-[r:SHOULD_EAT]->(:Dish)
        WITH p, collect(DISTINCT r.day) AS plannedDays
        RETURN p.name AS person, plannedDays,
            size(plannedDays) AS daysCovered,
            CASE WHEN size(plannedDays) = 7 THEN true ELSE false END AS hasWeeklyPlan
        """
        # Use the db_adapter to execute the query with parameters
        results = db_adapter.execute_query(query, {"name": intent_parameters["nome_utente"]})
        
        # Process the results directly from the adapter
        if results and len(results) > 0:
            intent_parameters["hasWeeklyPlan"] = results[0]["hasWeeklyPlan"]
    
    return intent_parameters