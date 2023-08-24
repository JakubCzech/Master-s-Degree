from __future__ import annotations

import logging
import threading
import time
from os import getenv
from os import system
from typing import List
from typing import Optional

import docker


CONTAINER_NAMES = {
    'simulation': 'simulation_py',
    'navigation': 'navigation_py',
    'visualization': 'visualization_py',
    'sender': 'sender_py',
}
NETWORK_NAME = 'agv_network'
ROS_DOMAIN_ID = int(time.time() * 100) % 100  # last two digits of current time


class Test:
    def __init__(self, test_title='Test', ros_domain_id: int = ROS_DOMAIN_ID):
        """Initialize the Test class."""
        self.logger = self._create_logger(ros_domain_id)
        self.client = docker.from_env()
        self.navigation_ready = threading.Event()
        self.nav2_error = threading.Event()
        self.remove_containers()
        self._create_containers(ros_domain_id)
        self.ros_domain_id = ros_domain_id
        self.docker_logger = threading.Thread(target=self._log_docker)
        self.logger.info(f'Tester {ros_domain_id} initialized')
        self.docker_logger.start()

    @classmethod
    def remove_containers(self) -> None:
        """Remove any running containers and networks."""
        commands = [
            'docker kill $(docker ps -a -q)',
            'docker rm $(docker ps -a -q)',
            'docker network rm $(docker network ls -q)',
        ]
        for cmd in commands:
            system(f'{cmd} 2> /dev/null > /dev/null')

    def run_test(self, test_title: str) -> list | None:
        """Run the test with the given title."""
        self.logger.info(f'Running test {test_title}')
        result = self.run_sender(test_title, self.ros_domain_id)
        self.remove_containers()

        if result:
            return result
        else:
            self.logger.error(f'Test {test_title} failed')
            return None

    def _create_containers(self, ros_domain_id: int) -> None:
        """Create and initialize containers for the test."""
        self.network = self.run_network(ros_domain_id)
        self.simulation = self.run_simulation(ros_domain_id)
        self.navigation = self.run_navigation(ros_domain_id)
        # Uncomment to run visualization (default rviz2)
        # self.visualization = self._run_visualization("rviz", ros_domain_id)

    def run_network(self, ros_domain_id) -> docker.models.networks.Network:
        try:
            network = self.client.networks.get(
                f'{NETWORK_NAME}_{ros_domain_id}',
            )
            network.remove()
        except docker.errors.NotFound:
            pass
        return self.client.networks.create(
            f'{NETWORK_NAME}_{ros_domain_id}', driver='bridge',
        )

    def run_simulation(self, ros_domain_id) -> docker.models.containers.Container:
        """Run simulation in docker container"""
        return self.client.containers.run(
            'ghcr.io/jakubczech/agv-simulation-ros2-gazebo:main',
            name=f"{CONTAINER_NAMES['simulation']}_{ros_domain_id}",
            volumes={'/dev': {'bind': '/dev', 'mode': 'ro'}},
            privileged=True,
            network=f'{NETWORK_NAME}_{ros_domain_id}',
            hostname='agv_py',
            environment={
                'DEBUG': 0,
                'MAPPING': 0,
                'SIMULATION': 'True',
                'GAZEBO_GUI': 0,
                'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                'ROS_DOMAIN_ID': ros_domain_id,
            },
            detach=True,
            remove=True,
            auto_remove=True,
            mem_limit='2g',
            cpuset_cpus='0,1,2,3,4,5,14,15,16,17',
        )
        # cmd = f"docker run -d --rm --name {CONTAINER_NAMES['simulation']}_{ros_domain_id} " \
        #         f"--network {NETWORK_NAME}_{ros_domain_id} --privileged " \
        #         f"-v /dev:/dev --hostname=agv_py" \
        #         f" -e DEBUG=0 -e MAPPING=0 -e SIMULATION=True -e GAZEBO_GUI=0 " \
        #         f"-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID={ros_domain_id} " \
        #         f"-m 8g --cpuset-cpus 1,2,3,4,5,6 " \
        #         f"ghcr.io/jakubczech/agv-simulation-ros2-gazebo:main"

    def run_navigation(self, ros_domain_id) -> docker.models.containers.Container:
        return self.client.containers.run(
            'ghcr.io/jakubczech/agv-navigation:main',
            name=f"{CONTAINER_NAMES['navigation']}_{ros_domain_id}",
            volumes={
                '/dev': {'bind': '/dev', 'mode': 'ro'},
                '/home/jakub/Documents/GitHub/Master-s-Degree/AGV-Navigation/src_files/src/agv_navigation/params/nav2_params_controller.yaml': {
                    'bind': '/root/workspace/src/agv_navigation/params/nav2_params_controller.yaml',
                    'mode': 'rw',
                },
            },
            privileged=True,
            network=f'{NETWORK_NAME}_{ros_domain_id}',
            hostname='agv_py',
            stdin_open=True,
            environment={
                'DEBUG': 'True',
                'MAPPING': 0,
                'SIMULATION': 'True',
                'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                'ROS_DOMAIN_ID': ros_domain_id,
            },
            detach=True,
            remove=True,
            auto_remove=True,
            mem_limit='2g',
            cpuset_cpus='6,7,8,9,10,11',
        )

    def run_sender(
        self, test_title: str, ros_domain_id: int,
    ) -> docker.models.containers.Container:
        command = f"/bin/bash -c 'source install/setup.bash &&  ros2 launch agv_waypoint_sender waypoint_sender.launch.py test_name:=\"{test_title}\"'"
        try:
            test = self.client.containers.run(
                'ghcr.io/jakubczech/agv-sender:main',
                name=f'{CONTAINER_NAMES["sender"]}_{ros_domain_id}',
                volumes={
                    '/dev': {'bind': '/dev', 'mode': 'ro'},
                    '/home/jakub/Documents/GitHub/Master-s-Degree/AGV-Waypoint_Sender/data': {
                        'bind': '/results',
                        'mode': 'rw',
                    },
                },
                privileged=True,
                network=f'{NETWORK_NAME}_{ros_domain_id}',
                hostname='agv_py',
                stdin_open=True,
                environment={
                    'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                    'ROS_DOMAIN_ID': ros_domain_id,
                },
                command=command,
                detach=True,
                remove=True,
                auto_remove=True,
                mem_limit='1g',
                cpuset_cpus='12',
            )
            for line in test.logs(stream=True, follow=True):
                self.logger.info(line.decode('utf-8').replace('\n', ''))
                if 'Test results:' in line.decode('utf-8'):
                    if 'FAILED' in line.decode('utf-8').split('Test results:'):
                        return False
                    else:
                        return line.decode('utf-8').split('Test results:')[1].split(',')
        except Exception:
            return False

    def run_visualization(
        self, tool: str, ros_domain_id: int,
    ) -> docker.models.containers.Container:
        return self.client.containers.run(
            'ghcr.io/jakubczech/agv-remote:main',
            name=f"{CONTAINER_NAMES['visualization']}_{ros_domain_id}",
            volumes={
                '/dev': {'bind': '/dev', 'mode': 'ro'},
                '/tmp/.X11-unix': {'bind': '/tmp/.X11-unix', 'mode': 'rw'},
            },
            privileged=True,
            network=f'{NETWORK_NAME}_{ros_domain_id}',
            hostname='agv_py',
            environment={
                'DEBUG': 0,
                'MAPPING': 0,
                'SIMULATION': 'True',
                'GAZEBO_GUI': 0,
                'TOOL': tool,
                'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                'ROS_DOMAIN_ID': ros_domain_id,
                'DISPLAY': getenv('DISPLAY'),
            },
            detach=True,
            remove=True,
            auto_remove=True,
            mem_limit='1g',
            cpuset_cpus='13',
        )

    def _create_logger(self, id: int) -> logging.Logger:
        """Create and return a logger with the given ID."""
        logger = logging.getLogger(f'Tester {id}')
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s %(levelname)s %(name)s %(message)s',
        )
        logger.setLevel(logging.INFO)
        return logger

    def _log_docker(self):
        """Log docker events."""
        logger = logging.getLogger('Docker')

        for line in self.navigation.logs(stream=True, follow=True):
            decoded_line = line.decode('utf-8')
            if 'Creating bond timer.' in decoded_line:
                logger.info('Navigation ready')
                self.navigation_ready.set()
            if 'FATAL' in decoded_line:
                logger.error(decoded_line)
                self.nav2_error.set()
            if 'ERROR' in decoded_line:
                logger.error(decoded_line)

    def __del__(self):
        """Destructor to ensure container removal upon deletion."""
        try:
            self._remove_containers(self.ros_domain_id)
        except Exception:
            pass
