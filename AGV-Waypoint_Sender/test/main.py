from __future__ import annotations

import logging

from test_class import Test
from utils import clear_screen
from utils import test
# from params import

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(name)s %(message)s',
)


# You can use a YAML or JSON file for configuration.
CONFIG_PATH = 'config/path_here.yaml'

# This can also be a dict if you use a JSON or a YAML configuration file.
CONFIG = {
    'planners': ['GridBased', 'SmacPlanner', 'ThetaStarPlanner'],
    'controllers': ['NEO', 'RPP'],
    'Kp': list(range(1, 6)),
    'Lookahead': [0.5, 1.0, 1.25],
    'Rotate_to_heading': [0.5, 1.0, 1.5],
    'num_of_repeat': 2,
}
TUNING_VFO_FILEPATH = 'vfo_tuning_fixed.text'
COMPARE_FILEPATH = 'compare.text'


def tuning_VFO():
    test_counter = 0
    test_number = len(CONFIG['planners']) * CONFIG['num_of_repeat'] * len(
        CONFIG['Kp'],
    ) * len(CONFIG['Lookahead']) * len(CONFIG['Rotate_to_heading'])
    info = []
    CONFIG['Kp'] = list(range(2, 6))
    for planner in CONFIG['planners']:
        for Ka_ in CONFIG['Kp']:
            for Lookahead_ in CONFIG['Lookahead']:
                for Rotate_to_heading_ in CONFIG['Rotate_to_heading']:
                    for _ in range(CONFIG['num_of_repeat']):
                        test_counter += 1
                        test_info = f'Test {test_counter}/{test_number}'
                        result= test(
                            planner, 'VFO', test_info, TUNING_VFO_FILEPATH,
                            Ka_, Lookahead_, Rotate_to_heading_,
                        )
                        info.append(result)
                        clear_screen()
    return info

def tuning_final():
    test_counter = 0
    test_number = len(CONFIG['planners'])  * len(CONFIG['Kp'])  * len(CONFIG['Lookahead']) * len(CONFIG['Rotate_to_heading'])
    info = []

    for planner in CONFIG['planners']:
        for Ka_ in CONFIG['Kp']:
            for Lookahead_ in len(CONFIG['Lookahead']):
                for Rotate_to_heading_ in CONFIG['Rotate_to_heading']:
                    if planner == 'GridBased' and Ka_ == 1 and Lookahead_ <= 1.25:
                        continue
                    test_counter += 1
                    test_info = f'Test {test_counter}/{test_number}'
                    result= test(
                        planner, 'VFO', test_info, TUNING_VFO_FILEPATH,
                        Ka_, Lookahead_, Rotate_to_heading_,
                    )
                    info.append(result)
                    clear_screen()
    return info


def compare_conf():
    test_counter = 0
    test_number = len(CONFIG['planners']) * \
        len(CONFIG['controllers']) 
    info = []
    for planner in CONFIG['planners']:
        for controller in CONFIG['controllers']:
            test_counter += 1
            test_info = f'Test {test_counter}/{test_number}'
            result = test(planner, controller, test_info, "vfo_2.text")
            info.append(result)
            clear_screen()
    return info


def test_final():
    test_counter = 0
    test_number = len(CONFIG['planners']) * \
        len(CONFIG['controllers']) * 10 
    info = []
    for planner in CONFIG['planners']:
        for controller in CONFIG['controllers']:
            for _ in range(10):
                test_counter += 1
                test_info = f'Test {test_counter}/{test_number}'
                result = test(planner, controller, test_info, "vfo_2.text")
                info.append(result)
                clear_screen()
    return info


if __name__ == '__main__':
    try:
        clear_screen()
        # try:
        #     final_info2 = compare_conf()
        # except:
        #     pass
        # try:
        #     final_info = tuning_VFO()
        # except:
        #     pass
        # try:
        #     final_info3 = tuning_VFO2()
        # except:
        #     pass
        final_info2 = tuning_final()


        clear_screen()
        logging.info('Done')
        print(final_info2)
    except KeyboardInterrupt:
        logging.warning('KeyboardInterrupt')
    finally:
        Test.remove_containers()
