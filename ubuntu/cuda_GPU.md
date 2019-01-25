###  Ubuntu 16.04只用內顯而不用獨顯 



在一台同時有intel內顯與nvidia獨立顯卡的ubuntu 16.04主機上，希望僅僅只用內顯來顯示畫面，獨顯是要用來做GPU加速。先用下面指令安裝nvidia顯卡驅動（這邊找到此顯卡的驅動版本是375.39，請依照自己的顯卡型號去找對應的）：

- apt-get install libcuda1-375 nvidia-375 nvidia-375-dev nvidia-libopencl1-375 

安裝完之後，會發現登入畫面還在，但是輸完帳號密碼之後不會直接進入圖形介面，而是直接跳回登入畫面。這是因為系統目前把顯示的工作直接丟給獨顯，而不是內顯。

 解決方式：先按 Ctrl-Alt-F1 跳到virtual console(Ctrl-Alt-F7回到視窗模式)，輸入：

1. prime-select intel
2. service lightdm restart

第一個指令是要求系統的顯示用intel的內顯，

第二個指令則是重新啟動登入畫面。都做完以後，應該就可以登入到圖形介面了。