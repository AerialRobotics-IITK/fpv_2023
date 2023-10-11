## Implementing an Autonomous FPV Racing Module

### Goal
  1. Getting an idea of classical path planning algorithms used in navigating an unknown environment
    We implemented these 2D algorithms using gui libraries in C++ and Python. This is organized under <a href="https://github.com/AerialRobotics-IITK/fpv_2023/tree/main/simulations_2d">/simulations_2d</a>
  2. Implementing a naive approach of hard-coding positions of the gate, and commanding the UAV to the centre coordinates; which are in turn obtained by classical detection methods. The source code and simulation demo for this can be found under <a href="https://github.com/AerialRobotics-IITK/fpv_2023/tree/main/naive_fpv_simul">/naive_fpv_simul</a>
  3. Implementing an intelligent system to navigate at efficient trajectories. We are re-implementing the approach demonstrated recenetly by UZH - <a href="https://www.nature.com/articles/s41586-023-06419-4">"Champion-level drone racing using deep reinforcement learning"</a>. We are aiming to mould the approach in order to make it more robust, at the expense of time or efficiency.


### Other References
- <a href="https://rpg.ifi.uzh.ch/docs/RSS20_Foehn.pdf">AlphaPilot: Autonomous Drone Racing - UZH</a>
- <a href="https://rpg.ifi.uzh.ch/docs/Loquercio21_Science.pdf">Learning High-Speed Flight in the Wild</a>

### Learning Resources
- [Google DeepMind - RL/DL Lecture Series](https://www.youtube.com/playlist?list=PLqYmG7hTraZCRwoyGxvQkqVrZgDQi4m-5)
