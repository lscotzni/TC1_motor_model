import numpy as np 

class PowertrainComponent(object):
    def __init__(self, name, failure_rate):
        self.name = name
        self.failure_rate = failure_rate

class Inverter(PowertrainComponent):
    def __init__(self, name, failure_rate, efficiency=1):
        super().__init__(name, failure_rate)
        self.efficiency = efficiency
        self.downstream_comp = ...

class Battery(PowertrainComponent):
    pass


if __name__ == '__main__':
    I1 = Inverter(name='inverter_1', failure_rate=0.001)

