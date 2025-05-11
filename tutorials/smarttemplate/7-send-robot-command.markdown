---
layout: page
title: "Tutorial on MRI-Guided Robot-Assisted Prostate Biopsy"
permalink: /tutorials/smarttemplate/7-send-robot-command.html
---

# Step 7: Send a robot command using SlicerROS2 publishers

- In the python console, create a publisher for `/desired_command` topic:

```python
pubCommand = rosNode.CreateAndAddPublisherNode('String', '/desired_command')
```

- Send "RETRACT" message and observe the robot fully retract the needle:

```python
pubCommand.Publish('RETRACT')
```

![Retract command](images/image24.jpg)

- Send "HOME" message and observe the robot go to its initial position:

```python
pubCommand.Publish('HOME')
```

![Home command](images/image25.jpg)

[⬅️ Previous: Read needle position](6-read-needle-position) | [Back to Table of Contents ↩️](index)
