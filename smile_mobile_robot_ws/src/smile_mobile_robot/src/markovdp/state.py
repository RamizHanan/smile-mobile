

class State(object):
    def __init__(self, state_name: str):
        self.name = state_name
        self.transisitons = []

    def __str__(self):
        return "{} - {}".format(self.name, str(self.transisitons))
