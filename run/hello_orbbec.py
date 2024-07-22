# ******************************************************************************
#  Copyright (c) 2023 Orbbec 3D Technology, Inc
#  
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.  
#  You may obtain a copy of the License at
#  
#      http:# www.apache.org/licenses/LICENSE-2.0
#  
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# ******************************************************************************

from pyorbbecsdk import *

class OBCameraParams:
    def __init__(self):
        self.l_intr_p = [0.0] * 4  # [fx, fy, cx, cy]
        self.r_intr_p = [0.0] * 4
        self.r2l_r = [0.0] * 9     # [r00, r01, r02; r10, r11, r12; r20, r21, r22]
        self.r2l_t = [0.0] * 3     # [t1, t2, t3]
        self.l_k = [0.0] * 5       # [k1, k2, p1, p2, k3]
        self.r_k = [0.0] * 5
        self.is_mirror = 0




