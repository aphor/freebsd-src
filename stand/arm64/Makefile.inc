#
# Shared settings for the arm64 loaders under stand/arm64.
#
# Its existence is load-bearing: bsd.init.mk includes exactly
# ${.CURDIR}/../Makefile.inc, one level up and no further, so without a
# Makefile.inc here a loader in stand/arm64/<name> never reaches
# stand/defs.mk and BOOTSRC, MK_FDT and the rest are undefined.  That is the
# same chaining stand/efi/Makefile.inc does.
#

.include "../Makefile.inc"
