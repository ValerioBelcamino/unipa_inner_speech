from clingo import Control
import os 
from ament_index_python.packages import get_package_share_directory
import json
from dotenv import load_dotenv

# Load environment variables from .env file
BASE_DIR = "/home/belca/Desktop/ros2_humble_ws/src"
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path)

def clingo_call():

    # Assuming your workspace structure is standard
    ws_dir = os.getenv("ROS2_WORKSPACE")
    source_dir = os.path.join(ws_dir, 'clingo_nodes', 'clingo_nodes')

    # Definisci il programma ASP
    program_data = ''
    program_logic = ''

    with open(os.path.join(source_dir, 'clingo.pl')) as f:
        program_logic = ''.join(f.readlines())
    print("Program logic loaded!")

    with open(os.path.join(source_dir, 'diet_data.pl')) as f:
        program_data = ''.join(f.readlines())
    print("Program data loaded!")

    program = program_data + program_logic

    print(program)

    # subprocess.run(['/usr/bin/clingo', os.path.join(source_dir, 'diet_data.pl'), os.path.join(source_dir, 'clingo.pl')])

    # Crea il solver
    ctl = Control()
    ctl.add("base", [], program)
    ctl.ground([("base", [])])

    answer_list = []

    # Risolvi e stampa i risultati
    with ctl.solve(yield_=True) as handle:
        for model in handle:
            if model.cost[0] < 100:
                print("\033[1;36mSoluzione trovata:\033[0m", model.symbols(shown=True))
                print("\033[1;33mCosto ottimale:\033[0m", model.cost)
                print(type(model.symbols(shown=True)))
                answer_list.append(str(model.symbols(shown=True)))
        print("\033[1;34mRisoluzione completata.\033[0m")

    print(f'\033[1;32mFound {len(answer_list)} solutions.\033[0m')

    answer_json = {'results': ','.join(answer_list)}
    answer_json = json.dumps(answer_json)

    print(f'\033[1;32mFound {answer_json} solutions.\033[0m')

    return answer_json
