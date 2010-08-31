import sys
import os

filename = sys.argv[1]

f = open(filename, 'r')
lines = []
for line in f:
  r = line.split()
  if len(r) >= 2:
    if r[0] == 'loadPlugin':
      plugin = r[1]
      l = len(plugin)
      words = []
      word = ''
      if l > 3:
        if plugin[0:3] == 'sot':
          plugin = plugin[3:]
          l = l - 3
          if plugin[0].isupper():
            plugin = plugin[0].lower() + plugin[1:]
          istart = 0
          i = 0
          for i in xrange(0, l-1):
             if plugin[i] == '$':
               word = plugin[istart:l].lower()
               words = words + [word]
               istart = l
               break
             if plugin[i].isupper():
               word = plugin[istart:i].lower()
               words = words + [word]
               istart = i
          if istart < l:
            word = plugin[istart:l]
            words = words + [word]
      if len(words) > 0:
        line = "loadPlugin " + words[0]
        for word in words[1:]:
          line = line + "-" + word
        line = line + '\n'
  lines = lines + [line]           
f.close()

try:
  os.makedirs("trans")
except OSError as exc: # Python >2.5
            pass

f = open("trans/"+filename, 'w')
for line in lines:
  f.write(line)
f.close()
