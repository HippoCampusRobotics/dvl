from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()
    sl.declare_arg('vehicle_name')
    sl.node(
        'dvl',
        'dvl_node',
        parameters=[sl.find('dvl', 'dvl.yaml')],
        namespace=sl.arg('vehicle_name'),
    )

    return sl.launch_description()
