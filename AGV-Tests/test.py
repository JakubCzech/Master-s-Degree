from os import getenv, system
import threading
import time
from typing import List
from params import get_params
import docker
import logging


CONTAINER_NAMES = {
    "simulation": "simulation_py",
    "navigation": "navigation_py",
    "visualization": "visualization_py",
    "sender": "sender_py",
}
NETWORK_NAME = "agv_network"
ROS_DOMAIN_ID = 11


class Test:
    def __init__(self, test_title="Test", ros_domain_id: int = ROS_DOMAIN_ID):
        self.logger = self.create_logger(ros_domain_id)
        self.client = docker.from_env()
        self.remove_containers(ros_domain_id)
        self.remove_network(ros_domain_id)
        self.create_containers(ros_domain_id)
        self.ros_domain_id = ros_domain_id
        self.docker_logger = threading.Thread(target=self.log_docker)
        self.logger.info(f"Tester {ros_domain_id} initialized")
        # self.docker_logger.start()

    def run_test(self, test_title: str) -> List:
        self.logger.info(f"Running test {test_title}")
        if result := self.run_sender(test_title, self.ros_domain_id):
            self.remove_containers(self.ros_domain_id)
            self.remove_network(self.ros_domain_id)
            return result
        else:
            self.remove_containers(self.ros_domain_id)
            self.remove_network(self.ros_domain_id)
            self.logger.error(f"Test {test_title} failed")
            return False

    # def run_waypoint_navigation(self, test_title: str, ros_domain_id: int):
    #     self.logger.info("Running waypoint navigation")
    #     self.logger.info("Waypoint navigation finished")

    def remove_containers(self, ros_domain_id):
        for name in CONTAINER_NAMES.values():
            try:
                if container := self.client.containers.get(f"{name}_{ros_domain_id}"):
                    try:
                        container.kill()
                    except docker.errors.APIError:
                        pass
                    container.remove()
            except docker.errors.NotFound:
                pass
            except docker.errors.APIError:
                pass

    def remove_network(self, ros_domain_id):
        try:
            network = self.client.networks.get(f"{NETWORK_NAME}_{ros_domain_id}")
            network.remove()
        except docker.errors.NotFound:
            pass

    def create_containers(self, ros_domain_id):
        self.network = self.run_network(ros_domain_id)
        self.simulation = self.run_simulation(ros_domain_id)
        self.navigation = self.run_navigation(ros_domain_id)
        # Uncomment to run visualization (default rviz2)
        # self.visualization = self.run_visualization("rviz", ros_domain_id)

    def run_network(self, ros_domain_id) -> docker.models.networks.Network:
        try:
            network = self.client.networks.get(f"{NETWORK_NAME}_{ros_domain_id}")
            network.remove()
        except docker.errors.NotFound:
            pass
        return self.client.networks.create(
            f"{NETWORK_NAME}_{ros_domain_id}", driver="bridge"
        )

    def run_simulation(self, ros_domain_id) -> docker.models.containers.Container:
        """Run simulation in docker container"""
        return self.client.containers.run(
            "ghcr.io/jakubczech/agv-simulation-ros2-gazebo:main",
            name=f"{CONTAINER_NAMES['simulation']}_{ros_domain_id}",
            volumes={"/dev": {"bind": "/dev", "mode": "ro"}},
            privileged=True,
            network=f"{NETWORK_NAME}_{ros_domain_id}",
            hostname="agv_py",
            environment={
                "DEBUG": 0,
                "MAPPING": 0,
                "SIMULATION": "True",
                "GAZEBO_GUI": 0,
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "ROS_DOMAIN_ID": ros_domain_id,
            },
            detach=True,
        )

    def run_navigation(self, ros_domain_id) -> docker.models.containers.Container:
        return self.client.containers.run(
            "ghcr.io/jakubczech/agv-navigation:main",
            name=f"{CONTAINER_NAMES['navigation']}_{ros_domain_id}",
            volumes={
                "/dev": {"bind": "/dev", "mode": "ro"},
                "/home/jakub/Documents/GitHub/Master-s-Degree/AGV-Navigation/src_files/src/agv_navigation/params/nav2_params_controller.yaml": {
                    "bind": "/root/workspace/src/agv_navigation/params/nav2_params_controller.yaml",
                    "mode": "rw",
                },
            },
            privileged=True,
            network=f"{NETWORK_NAME}_{ros_domain_id}",
            hostname="agv_py",
            stdin_open=True,
            environment={
                "DEBUG": 0,
                "MAPPING": 0,
                "SIMULATION": "True",
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "ROS_DOMAIN_ID": ros_domain_id,
            },
            detach=True,
        )

    def run_sender(
        self, test_title: str, ros_domain_id: int
    ) -> docker.models.containers.Container:
        try:
            command = f"/bin/bash -c 'source install/setup.bash &&  ros2 launch agv_waypoint_sender waypoint_sender.launch.py test_name:=\"{test_title}\"'"
            result = self.client.containers.run(
                "ghcr.io/jakubczech/agv-sender:main",
                name=f'{CONTAINER_NAMES["sender"]}_{ros_domain_id}',
                volumes={
                    "/dev": {"bind": "/dev", "mode": "ro"},
                },
                privileged=True,
                network=f"{NETWORK_NAME}_{ros_domain_id}",
                hostname="agv_py",
                stdin_open=True,
                environment={
                    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                    "ROS_DOMAIN_ID": ros_domain_id,
                },
                command=command,
                detach=False,
                stdout=True,
            )
            try:
                result = result.decode("utf-8")
                if result.split("Test results:"):
                    if result.split("Test results:")[1] == " FAILED Test results:":
                        return False
                    else:
                        return result.split("Test results:")[1].split(",")
            except Exception:
                return False

        except docker.errors.ContainerError as e:
            try:
                print(e.stderr.decode("utf-8"))
                if e.stderr.decode("utf-8").split("Test results:"):
                    if (
                        e.stderr.decode("utf-8").split("Test results:")[1]
                        == " FAILED Test results:"
                    ):
                        return False
                    else:
                        return (
                            e.stderr.decode("utf-8")
                            .split("Test results:")[1]
                            .split(",")
                        )
            except Exception:
                return False

    def run_visualization(
        self, tool: str, ros_domain_id: int
    ) -> docker.models.containers.Container:
        return self.client.containers.run(
            "ghcr.io/jakubczech/agv-remote:main",
            name=f"{CONTAINER_NAMES['visualization']}_{ros_domain_id}",
            volumes={
                "/dev": {"bind": "/dev", "mode": "ro"},
                "/tmp/.X11-unix": {"bind": "/tmp/.X11-unix", "mode": "rw"},
            },
            privileged=True,
            network=f"{NETWORK_NAME}_{ros_domain_id}",
            hostname="agv_py",
            environment={
                "DEBUG": 0,
                "MAPPING": 0,
                "SIMULATION": "True",
                "GAZEBO_GUI": 0,
                "TOOL": tool,
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "ROS_DOMAIN_ID": ros_domain_id,
                "DISPLAY": getenv("DISPLAY"),
            },
            detach=True,
        )

    def create_logger(self, id: int) -> logging.Logger:
        logger = logging.getLogger(f"Tester {id}")
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s %(levelname)s %(name)s %(message)s",
        )
        logger.setLevel(logging.INFO)
        return logger

    def log_docker(self):
        # create custom logger for docker
        logger = logging.getLogger("Docker")
        for line in self.navigation.logs(stream=True, follow=True):
            logger.info(line.decode("utf-8"))

    def __del__(self):
        self.remove_containers(self.ros_domain_id)
        self.remove_network(self.ros_domain_id)


if __name__ == "__main__":

    def test(params):
        Kp, Lookahead, Rotate_to_heading = params
        print(
            f"Kp: {Kp},Ka:{2*Kp} Lookahead: {Lookahead}, Rotate_to_heading: {Rotate_to_heading}"
        )
        params = get_params(Kp, Lookahead, Rotate_to_heading)
        with open(
            "/home/jakub/Documents/GitHub/Master-s-Degree/AGV-Navigation/src_files/src/agv_navigation/params/nav2_params_controller.yaml",
            "w",
        ) as f:
            f.write(params)
        test = Test("Test", 0)
        time.sleep(5)
        results = test.run_test(f"Test {Ka} {Lookahead} {Rotate_to_heading}")
        if results:
            print(results)
            with open("results.txt", "a") as f:
                f.write(f"{results}\n")
        else:
            print(f"Test failed {Ka} {Lookahead} {Rotate_to_heading}")

    for Ka in range(1, 6):
        for Lookahead in range(50, 150, 25):
            for Rotate_to_heading in range(10, 150, 25):
                try:
                    test((Ka, Lookahead / 100, Rotate_to_heading / 100))
                except Exception:
                    pass
