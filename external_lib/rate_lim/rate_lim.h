// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>

#ifndef RATE_LIM_H
#define RATE_LIM_H

#define RISE_LIM 70     //Предельная скорость роста сигнала в об/сек
#define FALL_LIM 70    //Предельная скорость спада сигнала в об/сек

class Rate_Limiter
{
    public:
        Rate_Limiter(int rising_lim, int falling_lim);
        int compute(int in, float dt);
        void updateConstants(int rising_lim, int falling_lim);

    private:
        int rising_lim_;
        int falling_lim_;

        int prev_out_;
        
};

#endif