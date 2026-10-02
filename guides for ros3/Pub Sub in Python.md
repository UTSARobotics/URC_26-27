# Pub/Sub in Python

This guide installs the Python bindings, creates a publisher and subscriber, and explains how to start them. The publisher sends messages; the subscriber prints the messages it receives.

## 1. Install the bindings

Run these commands with your virtual environment active. If you haven’t created one yet:

```sh
sudo apt install python3-full
python3 -m venv ~/.venv
source ~/.venv/bin/activate
```

Install Mirage first because `ros3` depends on it:

```sh
python -m pip install -e "$HOME/mirage/lang/python"
python -m pip install -e "$HOME/ros3/lang/python"
```

Run each command from the path shown, or use the full paths as above. Don’t run `python -m pip install -e .` from your home directory (`~`): `.` means “the current directory,” and your home folder isn’t a Python project.

Check that `ros3` imports:

```sh
python -c "import ros3; print('ros3 imported')"
```

## 2. Create the scripts

Keep the scripts in a convenient folder separate from the bindings:

```sh
mkdir -p ~/ros3-demo
cd ~/ros3-demo
```

Create `pub.py`:

```python
import ros3 as rose

node = rose.Node("py_publisher")
publisher = node.publisher("chatter", message_size=1024, rate=128)

while node.ok():
    publisher.publish(["hello from python"])
    rose.sleep(0.1)
```

Create `sub.py`:

```python
import ros3 as rose

node = rose.Node("py_subscriber")
subscriber = node.subscriber("chatter")

while node.ok():
    for message in subscriber:
        print(f"Received: {message}")
    rose.sleep(0.01)
```

The publisher sends messages; it does not print them. The subscriber prints the received messages. `publisher.publish(...)` encodes messages with Mirage automatically, and the subscriber yields decoded Python objects.

## 3. Set up each terminal

Create a helper script so you don’t have to retype the environment setup:

```sh
cat > ~/ros3-demo/setup-ros3.sh <<'SH'
source "$HOME/.venv/bin/activate"
export LD_LIBRARY_PATH="$HOME/ros3/build:$HOME/mirage/build:$HOME/quicksand/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
SH
```

In **each** terminal, run:

```sh
source ~/ros3-demo/setup-ros3.sh
cd ~/ros3-demo
```

This activates the virtual environment and sets the shared-library paths for that terminal.

## 4. Start the publisher, then the subscriber

In the first terminal, start the publisher:

```sh
python pub.py
```

It may appear to do nothing because the script doesn’t print anything. It should keep running and sending messages.

Then, in the second terminal, start the subscriber:

```sh
python sub.py
```

You should see messages such as:

```text
Received: ['hello from python']
```

Start the publisher first: the subscriber needs the publisher to create the `"chatter"` topic before it can attach to it. If you see an error saying the topic does not exist, check that the publisher is running and that both scripts use the exact same topic name. If you set `ROSE_BASE`, make sure it has the same value in both terminals.

Press **Ctrl+C** in each terminal to stop the scripts.
