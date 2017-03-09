def objectivize(cls):
    cls.objectivefx = dict()
    return cls

def objectivefx(argument1, argument2):
    def real_decorator(function):
        def wrapper(self):
            name = argument1
            minmax = argument2
            self.objectivefx[name] = [function, minmax]
        return wrapper
    return real_decorator