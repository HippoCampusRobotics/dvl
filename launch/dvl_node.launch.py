from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()
    sl.node('dvl', 'dvl_node', parameters=[sl.find('dvl', 'dvl.yaml')])

    return sl.launch_description()
