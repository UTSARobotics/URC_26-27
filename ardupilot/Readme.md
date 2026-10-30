After cloning ardupilot and installing the dependencies, you can run the simulation with the following commands.

Create 2 terminals

Within the ardupilot/Arducopter run:

```
../tools/autotest/sim_vehicle.py -v arducopter --model=json:127.0.0.1 --console --map -w
```

In the other terminal, run:

```
pip install -r requirements.txt
python mujoco_sitl_bridge.py
```
