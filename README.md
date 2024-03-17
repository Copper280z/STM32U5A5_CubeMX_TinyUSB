compile by setting CC and using bear to generate compile_commands.json
```CC=arm-none-eabi-gcc bear -- make```
If you don't want to use bear or set CC every time, uncomment the lines that set CC in the makefile around 75-80ish lines in.
