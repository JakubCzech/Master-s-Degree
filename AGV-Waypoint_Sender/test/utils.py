from __future__ import annotations

import logging
import os
import time

from params import get_params_NEO
from params import get_params_RPP
from params import get_params_VFO
from test_class import Test

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(name)s %(message)s',
)


PARAM_FILE_PATH = '/home/jakub/Documents/GitHub/Master-s-Degree/AGV-Navigation/src_files/src/agv_navigation/params/nav2_params_controller.yaml'


PARAMS = {
    'NEO': get_params_NEO,
    'RPP': get_params_RPP,
    'VFO': get_params_VFO,
}


def clear_screen():
    os.system('clear')


def test(planner: str, controller: str, test_info: str, result_file_path: str, Kp=None, Lookahead=None, Rotate_to_heading=None):
    if Kp:
        name = f"{planner}_{controller}_{test_info.replace('/', '_')}_{Kp}_{Lookahead}_{Rotate_to_heading}"
    else:
        name = f"{planner}_{controller}_{test_info.replace('/', '_')}"

    test_info = f'{test_info} {name}'
    logging.info(test_info)

    with open(PARAM_FILE_PATH, 'w') as f:
        f.write(PARAMS[controller](planner, Kp, Lookahead, Rotate_to_heading))

    test_instance = Test(name)
    #  wait for navigation  or nav2 error
    while not test_instance.navigation_ready.is_set() and not test_instance.nav2_error.is_set():
        time.sleep(.00001)

    if test_instance.nav2_error.is_set():
        logging.error(f'Test failed {name}, test: {test_info}')
        return {'name': name, 'status': 'FAILED', 'reason': 'NAV2 ERROR'}

    time.sleep(1)
    try:
        results = test_instance.run_test(name)
    except Exception as e:
        logging.info(f'Exception during test run: {e}')
        time.sleep(1)
        results = test_instance.run_test(name)

    clear_screen()

    if results:
        logging.error(f'Testing {name}, test: {test_info}. Results: {results}')
        with open(result_file_path, 'a') as f:
            f.write(f'{results}\n')
        return {'name': name, 'status': 'SUCCESS', 'results': results}
    else:
        return {'name': name, 'status': 'FAILED', 'reason': 'Test execution failed.'}
