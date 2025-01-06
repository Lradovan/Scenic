## **Installation**

To interface with Isaac Sim, follow these steps:

1. Follow the instructions [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html#) to install Isaac Sim for a workstation.

2. Follow the instructions [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html#python-environment-installation) to install Isaac Sim Python packages into your Scenic Python Virtual Environment.

3. To use NVIDIA assets you need to start a local Nucleus server. In the Omniverse Launcher, navigate to the "Nucleus" Tab. If Nucleus hasn't been installed yet, click the install button. Next, click "Add Local Nucleus Server" and follow the on-screen instructions to start a localhost server.

4. To test that everything is working, try the following:
```python
import scenic
from scenic.simulators.isaacsim import IsaacSimSimulator
scenario = scenic.scenarioFromFile("Scenic/examples/isaacsim/create3/basic.scenic", 
                                   model='scenic.simulators.isaacsim.model')
scene, _ = scenario.generate()
simulator = IsaacSimSimulator()
simulation = simulator.simulate(scene, maxSteps=1000)
```

5. Note: I am unable to interrupt the Python script using Ctrl+C (nothing happens). So I either set `maxSteps` when calling `simulate`, or manually kill the process.