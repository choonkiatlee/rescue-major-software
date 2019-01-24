# Robot Visions (by Isaac Asimov)
#
# I suppose I should start by telling you who I am. I am a very junior member of the Temporal Group. The Temporalists
# (for those of you who have been too busy trying to survive in this harsh world of 2030 to pay much attention
# to the advance of technology) are the aristocrats of physics these days.
# They deal with that most intractable of problems—that of moving through time at a speed different from the steady
# temporal progress of the Universe. In short, they are trying to develop time-travel.
# And what am I doing with these people, when I myself am not even a physicist, but merely a—? Well, merely a merely.
# Despite my lack of qualification, it was actually a remark I made some time before that inspired the Temporalists
# to work out the concept of VPIT (“virtual paths in time”).
# You see, one of the difficulties in traveling through time is that your base does not stay in one place relative
# to the Universe as a whole. The Earth is moving about the Sun; the Sun about the Galactic center; the Galaxy about
# the center of gravity of the Local Group—well, you get the idea. If you move one day into the future or
# the past—just one day—Earth has moved some 2.5 million kilometers in its orbit about the Sun. And the Sun has moved
# in its journey, carrying Earth with it, and so has everything else.
# Therefore, you must move through space as well as through time, and it was my remark that led to a line of argument
# that showed that this was possible; that one could travel with the space-time motion of the Earth not in a literal,
# but in a “virtual” way that would enable a time-traveler to remain with his base on Earth wherever he went in time.
# It would be useless for me to try to explain that mathematically if you have not had Temporalist training. Just
# accept the matter.
# It was also a remark of mine that led the Temporalists to develop a line of reasoning that showed that travel
# into the past was impossible. Key terms in the equations would have to rise beyond infinity when the temporal signs
# were changed.
# It made sense. It was clear that a trip into the past would be sure to change events there at least slightly, and no
# matter how slight a change might be introduced into the past, it would alter the present; very likely drastically.
# Since the past should seem fixed, it makes sense that travel back in time is impossible.
# The future, however, is not fixed, so that travel into the future and back again from it would be possible. I was
# not particularly rewarded for my remarks. I imagine the Temporalist team assumed I had been fortunate in my
# speculations and it was they who were entirely the clever ones in picking up what I had said and carrying it through
# to useful conclusions. I did not resent that, considering the circumstances, but was merely very glad—delighted,
# in fact—since because of that (I think) they allowed me to continue to work with them and to be part of the project,
# even though I was merely a—well, merely.
# Naturally, it took years to work out a practical device for time travel, even after the theory was established, but
# I don’t intend to write a serious treatise on Temporality. It is my intention to write of only certain parts
# of the project, and to do so for only the future inhabitants of the planet, and not for our contemporaries.
# Even after inanimate objects had been sent into the future—and then animals—we were not satisfied. All objects
# disappeared; all, it seemed, traveled into the future. When we sent them short distances into the future—five minutes
# or five days—they eventually appeared again, seemingly unharmed, unchanged, and, if alive to begin with, still alive
# and in good health.
# But what was wanted was to send something far into the future and bring it back.


import dataset
import train
from config import *

if MAKE_DATASET:
    dataset.createDataset()
if TRAIN:
    train.train()

