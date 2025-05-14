from datetime import datetime, timedelta



def get_day_of_the_week(intent_parameters: dict, action_name: str) -> dict:
    # print(f"\033[95m[get_day_of_the_week] parameters = {{'intent_parameters': {intent_parameters}, 'action_name': {action_name}}}\033[0m")

    # the day of the week is a valid parameter only for the meal preparation case, let's filter out other action_names
    if action_name == 'SubstituteDish':

        days_of_the_week = {
                0: "lunedi",
                1: "martedi",
                2: "mercoledi",
                3: "giovedi",
                4: "venerdi",
                5: "sabato",
                6: "domenica",
            }
                
        if intent_parameters['giorno'] in ['oggi', '']:
            intent_parameters['giorno'] = days_of_the_week[datetime.today().weekday()]

        elif intent_parameters['giorno'] == 'domani':
            domani = datetime.today() + timedelta(days=1)
            intent_parameters['giorno'] = days_of_the_week[domani.weekday()]

        elif intent_parameters['giorno'] == 'ieri':
            ieri = datetime.today() + timedelta(days=-1)
            intent_parameters['giorno'] = days_of_the_week[ieri.weekday()]

    return intent_parameters



def get_next_meal(intent_parameters: dict, action_name: str) -> dict:
    # print(f"\033[95m[get_next_meal] parameters = {{'intent_parameters': {intent_parameters}, 'action_name': {action_name}}}\033[0m")

    # the day of the week is a valid parameter only for the meal preparation case, let's filter out other action_names
    if action_name == 'SubstituteDish' and intent_parameters['pasto'] == '':

        current_time = datetime.now().time() #11:34:30.263342

        colazione = datetime.strptime("11:00", "%H:%M").time()
        pranzo = datetime.strptime("14:00", "%H:%M").time()
        cena = datetime.strptime("22:00", "%H:%M").time()

        if current_time < colazione:
            intent_parameters['pasto'] = 'colazione'
        elif current_time < pranzo:
            intent_parameters['pasto'] = 'pranzo'
        elif current_time < cena:
            intent_parameters['pasto'] = 'cena'

    return intent_parameters