# COMP0250 Coursework 1 Group 21

---

Authors: Jiale Li (jiale.li.24@ucl.ac.uk), Renkai Liu (renkai.liu.20@ucl.ac.uk), Zhengyang Zhu(zhengyang.zhu.21@ucl.ac.uk)

Description: This package forms the Group 21's solution for COMP0250 Coursework 1.

## Building the Code

To build the code, run the following commands:

```bash
cd ~/comp0250_s25_labs
catkin build
```

## Running the Code

To run our implementation, use the following command:

```bash
source devel/setup.bash
roslaunch cw1_team_21 run_solution.launch
```

Then, in a separate terminal, you can trigger each task:

```bash
source devel/setup.bash

# For Task 1: Pick and Place
rosservice call /task 1

# For Task 2: Object Detection & Colour Identification
rosservice call /task 2

# For Task 3: Planning and Execution
rosservice call /task 3
```

## Task Contributions

- **Task 1**: Jiale Li (33%), Renkai Liu (33%), Zhengyang Zhu (34%)
- **Task 2**: Jiale Li (34%), Renkai Liu (33%), Zhengyang Zhu (33%)
- **Task 3**: Jiale Li (33%), Renkai Liu (34%), Zhengyang Zhu (33%)
- 
- **Jiale Li**: 20 hours
- **Renkai Liu**: 20 hours
- **Zhengyang Zhu**: 20 hours

## License
LICENSE: MIT.  See [LICENSE](LICENSE)

DISCLAIMER:

THIS INFORMATION AND/OR SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS INFORMATION AND/OR
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (C) 2025 Jiale Li, Renkai Liu, and Zhengyang Zhu except where specified
