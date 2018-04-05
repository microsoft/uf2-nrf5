target extended-remote localhost:3333
define rst
  #set {int}(0x20008000-4) = 0xf02669ef
  monitor reset halt
  continue
end
define irq
  echo Current IRQ: 
  p (*(int*)0xE000ED04 & 0x1f) - 16
end
#echo Use 'rst' command to re-run program from start (set your breakpoints first!).\n
