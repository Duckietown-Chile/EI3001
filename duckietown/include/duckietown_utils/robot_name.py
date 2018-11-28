from .detect_environment import on_duckiebot

__all__ = [
    'ThisIsNotADuckiebot',
    'get_current_robot_name',
]


class ThisIsNotADuckiebot(Exception):
    pass


def get_current_robot_name():
    """
        To be used by all nodes that want to know what is the current
        robot name.

        Raises ThisIsNotADuckiebot if this is not a Duckiebot.
    """
    if not on_duckiebot():
        msg = 'This is not a Duckiebot. You will have to set the name in a different way.'
        raise ThisIsNotADuckiebot(msg)
    import socket
    robot_name = socket.gethostname()

    return robot_name
