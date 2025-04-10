s = serialport("COM16", 460800);
configureTerminator(s, "LF");
s.Timeout = 30;
flush(s);
writeline(s, "LED=1");
response = readline(s);
clear s;