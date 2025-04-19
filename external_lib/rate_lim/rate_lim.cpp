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
/**
  * @brief  Функция динамического ограничения скорости изменения сигнала (https://studopedia.ru/20_27817_blok-ogranicheniya-skorosti-izmeneniya-signala-Rate-Limiter.html)
  *
  * @param	Rate_Limiter_Param		-- Структура, содержащая параметры динамического ограничителя скорости изменения
  * 								   сигнала - частоту считывания, верхний и нижний пороги скорости
  *
  * @retval cur_out					-- Выходной сигнал с учетом ограничения по скорости
  *
  *	@note	Для корректной работы требуется достаточно точно выдерживать время между двумя последовательными считываниями,
  *			обратное частоте дискретизации. Поэтому данная функция предполагает работу по таймеру - в обработчике
  *			прерываний или в цикле while(1) по флагу (например, окончания преобразования на АЦП с запуском по таймеру)
  *
  *	@note	Обратите внимание, что выражение up_lim / f подразумевает существование ошибки округления, в связи с чем
  *			фактическая скорость изменения сигнала может оказаться чуть больше предела. В связи с этим рекомендуется
  *			задавать верхний и нижний пороги с поправкой на это.
  *
  */


#include "rate_lim.h"

Rate_Limiter::Rate_Limiter(int rising_lim, int falling_lim):
    rising_lim_(rising_lim),
    falling_lim_(falling_lim)
{
}

int Rate_Limiter::compute(int in, float dt){
	int cur_out;
//Считаем скорость изменения сигнала (производную от сигнала)
	int rate = (int) ((in - prev_out_) / dt);  // rate = (input_sig - prev_out) / dt , где dt = 1/freq
//Если скорость роста сигнала выше допустимой, то выходной сигнал считаем как интеграл от максимальной скорости роста со смещением prev_out
	if(rate > rising_lim_){
		//Из-за округления фактическая скорость может оказаться выше up_lim, что необходимо учитывать при задании up_lim
		cur_out = (int)(rising_lim_ * dt) + prev_out_;
	}
//Если скорость спада сигнала выше допустимой (по абсолютной величине), то выходной сигнал считаем как интеграл от максимальной скорости спада со смещением prev_out
	else if(rate < - falling_lim_){
		cur_out = - (int)(falling_lim_ * dt) + prev_out_;
	}
//Если скорость не выходит за пределы, то выходной сигнал равен входному
	else {
		cur_out = in;
	}
//Обратите внимание, что данная логика неприменима для ограничения скорости снизу, так как в таком случае
//выходной сигнал всегда будет опережать входной, и потому расти выше максимального значения

	prev_out_ = cur_out;
	return cur_out;
}


void Rate_Limiter::updateConstants(int rising_lim, int falling_lim)
{
    rising_lim_ = rising_lim;
    falling_lim_ = falling_lim;
}



