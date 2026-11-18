# Goal

A simulated arm and hand that learns, end to end, to pick up a ball, throw it, and hit a target, shown in renders good enough for the PI's website.

## Done means
- One policy, trained here, does the whole sequence from a resting ball: reach, grasp, lift, wind up, release, hit.
- A measured hit rate on held-out target positions and distances (report it with every checkpoint), and a short clip of the policy at each milestone.
- A high-quality offline render: 4K, 60 fps, cinematic lighting, motion blur and depth of field, a few camera angles, 20-40 s, suitable for a portfolio page. Plus stills.
- A live view in the lab: a WebGL build of the current best policy, served on the seat's port, so it hangs on a wall screen in Agentworld and updates when a better checkpoint lands.

## Order
1. Upgrade the project to Unity 2022.3 LTS (installed at /Applications/Unity/Hub/Editor/2022.3.62f3) and a current ML-Agents release, with the Python side pinned in a uv-managed environment.
2. Headless training: a batchmode/server build with many parallel environments, `mlagents-learn` driven from the command line, TensorBoard on the seat's port.
3. Curriculum: grasp, then lift, then throw anywhere, then hit near targets, then far and varied targets. Rewards and resets designed for each stage; record what worked.
4. Rendering: URP quality pass (lighting, materials, post-processing), then Unity Recorder captures, then the WebGL live view.
