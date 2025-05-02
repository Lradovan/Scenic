## **Installation**

To interface with Isaac Sim, follow these steps:

1. Follow the instructions [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html#) to install Isaac Sim for a workstation.

2. Follow the instructions [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html#python-environment-installation) to install Isaac Sim Python packages into your Scenic Python Virtual Environment.

3. To test that everything is working, try the following:
```python
import scenic
from scenic.simulators.isaacsim import IsaacSimSimulator
scenario = scenic.scenarioFromFile("Scenic/examples/isaacsim/create3/basic.scenic", 
                                   model='scenic.simulators.isaacsim.model')
scene, _ = scenario.generate()
simulator = IsaacSimSimulator()
simulation = simulator.simulate(scene, maxSteps=1000)
```