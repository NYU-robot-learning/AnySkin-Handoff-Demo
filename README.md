# AnySkin Handover Demo

0. Start Hello Robot and home it:

```bash
stretch_robot_home.py
```

1. Install the environment in `conda_env.yml`:

```bash
conda env create -f conda_env.yml
```

2. In one terminal pane, run the robot server without activating any Conda environment:

```bash
cd robot-server
python3 start_server.py
```

3. In another terminal pane, start the handover demo by running:

```bash
python3 run.py
```



