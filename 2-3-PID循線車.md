# PID 循線車

PID 演算法利用三種運算—比例、積分、微分，透過誤差值（稱為 error），計算出新的輸出值，而 PID 三種運算的權重分別由 Kp、Ki、Kd 三個參數進行控制。

$$ MV(t) = K_pe(t)+K_i\int_0^te(τ)dτ+Kd{d \over dt}e(t) $$

PID 演算法函數式（擷取自維基百科）

「比例控制」考慮現在的誤差，將 error 值乘上一個固定比例 Kp 後反應到輸出值上，可達到「偏移越多，修正越多」的效果。

積分控制」是考慮過去的誤差，將過去的 error 總和乘上固定比例 Ki 後反應到輸出上，可達到「過去偏移越多，現在修正越多」的效果。

「微分控制」考慮未來的趨勢，藉由計算上次 error 與此次 error 的差，可得到目前移動的趨勢，並乘上一個固定比例 Kd 反應到輸出值上，由於考慮的是未來的趨勢，微分運算可以避免晃動，避免轉彎後過衝的狀況。

``` c
常數 SPEED = 60;

void loop() {
    float error = get_value();
    float turn = PID運算(error);
    設定馬達轉速(SPEED+turn, SPEED-turn);
}
```

此為PID運算的順序，設定一個常數 SPEED 為車子前進的速度，並將PID運算得到的結果反應到左右馬達上。P、I、D 三個運算的結果加總為 turn 值，成為最終修正值。

$$ 最終修正值 = P 運算 + I 運算 + D 運算 $$

接下來分別介紹這三種運算。

## （ㄧ）P 比例運算

比例運算是將 error 乘上比例權重 Kp 後，反應到馬達轉速。基於先前邏輯判斷循線的經驗，我們使用（80,
0）的轉速計算出合適的 Kp 值。轉彎的數值為 80 和 0，由於馬達轉速是由 SPEED±turn 得出，所以SPEED值就是$(80-0)÷2=40$，為避免太慢造成馬達停擺，所以增加10(變為50)，馬達範圍除以感測值範圍：$(80 - 0) ÷ 200 = 0.4$，最後 Kp 值預設為 0.4。

``` c
常數 Kp = 0.4;

float P運算 (float error) {
    return error * Kp;
}
```

## （二）積分運算

積分運算會累加一小段時間差內的 error 值，並乘上積分權重 Ki，反應到馬達轉速。實務上為了避免在成功過彎後仍然被積分影響，因此在每次累加前都會先將舊的積分乘上 3/4 減少累積值的影響。

``` c
全域變數 Integral;
常數 Ki = 0.5;

float I運算 (float error) {
    Integral *= 3 / 4;
    Integral += error;
    return Integral * Ki;
}
```

## （三）D 微分運算

微分運算可以得到目前的趨勢，將其乘上微分權重 Kd 反應到馬達轉速。將此次 error 與上次 error 值的差傳回，並且記錄此次 error 值。

``` c
全域變數 last_error;
常數 Kd = 40;

float D運算 (float error) {
    float 差 = error - last_error;
    last_error = error;
    return 差 * Kd;
}
```

