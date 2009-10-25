// INDRETTI.CPP - A robot "driver" for RARS, by M. Inman, March 1995
// written for ver. 0.39 of RARS

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

// This is the track setup information (Data file to be read at run-time)

//TSI:! Speed2.trk
//TSI:
//TSI:CourseNSeg    = 18
//TSI:CourseWidth   = 150.0
//TSI:CourseRadii   =             0             300               0             450               0             550               0             480               0             350               0            -200               0             350               0            -200               0             710 
//TSI:CourseLengths =          2770            1.65             450             .55             600             .48          3350.0            2.10             300              .5             300              .5             300              .5             300              .5           400.0            1.54 
//TSI:
//TSI:CornerSpeed =     6.329393951     5.912184787     7.258597876     8.503967327     8.841662702     7.480799852     5.832880753     5.774993855    12.695238414     7.136596854    13.763743118     6.202806072    12.958238301     7.199428595    13.207036476     6.212034935     5.120371962     5.829358225
//TSI:SteerGain   =     1.031690515     0.779369698     1.080470164     0.895366396     0.830030990     0.989639183     0.971582806     0.953855875     0.906739134     0.898825729     1.170021835     0.973322322     0.872898064     0.961963175     0.924148526     0.859151217     0.951598482     1.030468993
//TSI:SteerDamp   =     1.650265225     1.989676192     1.612945259     1.507956567     1.666767877     1.382440558     1.465342753     1.832998034     1.822962284     1.634381182     1.645862509     1.929993865     1.450396257     1.753377870     1.586793485     1.647315881     1.966455698     1.525302186
//TSI:SteerBias   =     9.224241718     8.676539506     9.346831208    11.923922492    10.507811225     9.498229096    12.635159306    10.306748299    11.495980023     9.248916750     9.901591347     9.426495951    10.712544922     9.988766542     9.786034935     9.409648976     9.717768804     9.437670153
//TSI:ExtraSpace  =    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000
//TSI:CornerSetup =     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000
//TSI:BrakeFactor =     0.000345057     0.000416025     0.000402947     0.000381282     0.000416151     0.000454610     0.000369081     0.000383844     0.000373214     0.000448772     0.000369957     0.000387683     0.000486691     0.000337756     0.000392489     0.000403302     0.000372547     0.000408188
//TSI:AccelFactor =     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000
//TSI:STEFactor   =     3.600000000     3.600000000     3.600000000     3.600000000     3.600000000     3.600000000     3.600000000     3.600000000     3.600000000     3.600000000     3.600000000     3.600000000     3.600000000     3.600000000     3.600000000     3.600000000     3.600000000     3.600000000
//TSI:STEFactor2  =     2.200000000     2.200000000     2.200000000     2.200000000     2.200000000     2.200000000     2.200000000     2.200000000     2.200000000     2.200000000     2.200000000     2.200000000     2.200000000     2.200000000     2.200000000     2.200000000     2.200000000     2.200000000
//TSI:
//TSI:TweakMode     = Segment
//TSI:TweakNLaps    = 100
//TSI:TweakSStep    = 0.01
//TSI:TweakEStep    = 0.002
//TSI:SegmentList   =             0               1               2               3               4               5               6               7               8               9              10              11              12              13              14              15              16              17
//TSI:ParameterList = CornerSpeed BrakeFactor SteerGain SteerDamp SteerBias
//TSI:
//TSI:/*
//TSI:
//TSI:! Stef2.trk
//TSI:
//TSI:CourseNSeg    = 16
//TSI:CourseWidth   = 90.0
//TSI:CourseRadii   =   0 120   0 200   0 300     -200  0 110   0 120 -120   0 -50  0 125
//TSI:CourseLengths = 770  .9 200 .25 200 3.0 2.094395 90 2.9 150  .7   .7 140 1.9 70 3.2
//TSI:
//TSI:CornerSpeed =    12.593450307     5.928000000     7.341919180     6.460516332     5.467798452     6.274155284     5.650403828     6.283490304     5.860587239    15.004681077    10.060070422     5.700000000     5.579386176     7.500811142     5.750183712     6.165120000
//TSI:SteerGain   =     0.790902461     0.933390011     0.782144587     0.846714613     0.847974400     0.899891200     1.052427494     0.972169908     0.846714613     0.881893376     0.865280000     0.832000000     0.815113761     0.881893376     0.832000000     0.775084412
//TSI:SteerDamp   =     1.481750574     1.466636718     1.175774116     1.481750573     1.414000000     1.621096404     1.439442480     1.382440558     1.718310313     1.352779011     1.470560000     1.455560288     1.410653630     1.414000000     1.529382400     1.441148800
//TSI:SteerBias   =     8.903731200     8.525754436     8.879977083     9.053206712     9.615968520     9.515011240     8.736000000     9.341405917     8.890503440     9.173523159     8.222253312     9.085440000     8.736000000     8.725656576     9.085440000     9.085440000
//TSI:ExtraSpace  =    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000
//TSI:CornerSetup =     0.935000000     0.865280000     0.800000000     0.831014912     0.800000000     0.840000000     0.800000000     0.700000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000
//TSI:BrakeFactor =     0.000384639     0.000403902     0.000373656     0.000373656     0.000342046     0.000412270     0.000380831     0.000349440     0.000408188     0.000355727     0.000396413     0.000336000     0.000352517     0.000349440     0.000349026     0.000349440
//TSI:AccelFactor =     0.000800000     0.000800000     0.000832000     0.000832000     0.000814395     0.000799053     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000
//TSI:STEFactor   =     3.000000000     3.000000000     3.120000000     3.055753488     2.877788659     3.120000000     3.116305920     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000
//TSI:STEFactor2  =     1.560000000     1.687296000     1.622400000     1.560000000     1.560000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000
//TSI:
//TSI:TweakMode     = Segment
//TSI:TweakNLaps    = 25
//TSI:TweakSStep    = 0.04
//TSI:TweakEStep    = 0.009
//TSI:SegmentList   =         0               1              2                3               4               5                                                   8           9                10            11               12              13              14              15
//TSI:ParameterList = CornerSpeed BrakeFactor SteerBias SteerGain SteerDamp
//TSI:
//TSI:/*
//TSI:
//TSI:! ANew.trk
//TSI:
//TSI:CourseNSeg    = 23
//TSI:CourseWidth   = 120.0
//TSI:CourseRadii   =    0      300   0     -200   0      400     -250      350   0       750    0      250   0      280   0    -200   -1100     -150   0     -450     -650   0      390 
//TSI:CourseLengths = 2455 1.919862 300 0.698132 400 0.698132 0.698132 1.570796 700  0.349066 1700 1.570796 900 3.141593 550 1.37881 0.78539 0.872665 400 0.785398 0.977384 640 3.211406
//TSI:
//TSI:CornerSpeed =     6.134120742     6.329393951     8.341443045     6.165120000     8.672480887     9.298291679     7.635775241     6.922527495     5.928000000     5.693251200     5.809440000     6.041817600     5.700000000     6.534829916     5.579386176     5.809440000     7.212318405     5.700000000     5.985471841     6.283490304     6.102235776     5.358442483     5.928000000
//TSI:SteerGain   =     0.934496455     0.799052800     0.935886848     0.832000000     0.906465299     0.935604210     0.881627044     1.052745423     0.847974400     0.800000000     0.865018685     0.823513600     0.782835256     0.800000000     0.800000000     0.832000000     0.800000000     0.807043328     0.800000000     0.847974400     0.807043328     0.944960252     0.832000000
//TSI:SteerDamp   =     1.455560288     1.455560288     1.470560000     1.481750573     1.455560288     1.437303983     1.441148800     1.426449082     1.384079308     1.470560000     1.513782700     1.470560000     1.455120709     1.414000000     1.470560000     1.440713573     1.498794752     1.470560000     1.414000000     1.412325824     1.441148800     1.441148800     1.414000000
//TSI:SteerBias   =     9.630275666     8.057808246     8.212518164     8.982121074     9.071942285     8.903731200     8.390054400     8.558694493     9.085440000     8.400000000     9.448857600     9.074682839     8.551143444     8.733361728     8.736000000     8.304475845     9.085440000     8.736000000     8.473954944     8.736000000     8.820695345     8.736000000     8.400000000
//TSI:ExtraSpace  =    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000
//TSI:CornerSetup =     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000
//TSI:BrakeFactor =     0.000328890     0.000335501     0.000335602     0.000342451     0.000359711     0.000348921     0.000396413     0.000336000     0.000349440     0.000336000     0.000332179     0.000342046     0.000336000     0.000342348     0.000400619     0.000389063     0.000363308     0.000336000     0.000335501     0.000352517     0.000349334     0.000349229     0.000335205
//TSI:AccelFactor =     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000
//TSI:STEFactor   =     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000
//TSI:STEFactor2  =     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000
//TSI:
//TSI:TweakMode     = Segment
//TSI:TweakNLaps    = 25
//TSI:TweakSStep    = 0.04
//TSI:TweakEStep    = 0.009
//TSI:SegmentList   = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22
//TSI:ParameterList = CornerSpeed BrakeFactor SteerBias SteerGain SteerDamp
//TSI:
//TSI:/*
//TSI:
//TSI:! V03.TRK
//TSI:
//TSI:
//TSI:CourseNSeg    = 10
//TSI:CourseWidth   = 110.0
//TSI:CourseRadii   =             0         226.436           -1400         226.436               0          163.22        -216.092         201.149               0         352.874
//TSI:CourseLengths =        1310.0             2.9             .95        2.107129          561.37             1.0             1.3            2.07           252.5             .54
//TSI:
//TSI:CornerSpeed =     5.785333367     6.885801458     6.004320785     6.338811158    11.500000000     9.942664482     7.397962955     5.679777562     7.800843587     7.212318405
//TSI:SteerGain   =     0.898825729     0.819372835     0.943556379     1.011056705     0.800000000     0.852405175     1.019958210     1.137961810     0.831014912     0.914722211
//TSI:SteerDamp   =     1.896453505     1.632939223     1.527571611     1.523956463     1.414000000     1.392539891     1.521692409     1.494796142     1.498794752     1.620117409
//TSI:SteerBias   =     9.330345692    11.004633998    10.690421140    11.819858566     8.400000000     8.866784612     8.893189182    10.204701286     9.826811904     9.151813116
//TSI:ExtraSpace  =    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000
//TSI:CornerSetup =     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000
//TSI:BrakeFactor =     0.000341641     0.000436993     0.000355199     0.000368969     0.000336000     0.000376946     0.000440840     0.000344953     0.000362448     0.000473355
//TSI:AccelFactor =     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000
//TSI:STEFactor   =     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000
//TSI:STEFactor2  =     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000
//TSI:
//TSI:SegmentList   =             0               1               2               3                               5               6               7               8               9
//TSI:TweakMode     = Segment
//TSI:TweakNLaps    = 10
//TSI:TweakSStep    = 0.04
//TSI:TweakEStep    = 0.009
//TSI:ParameterList = CornerSpeed BrakeFactor SteerBias SteerGain SteerDamp
//TSI:
//TSI:/*
//TSI:
//TSI:! Oval2.trk
//TSI:
//TSI:
//TSI:CourseNSeg    = 4
//TSI:CourseWidth   = 100.0
//TSI:CourseRadii   =             0             400               0             400
//TSI:CourseLengths =           600       3.1415927             600       3.1415927
//TSI:
//TSI:CornerSpeed =     5.000000000     6.800000000     5.000000000     6.800000000
//TSI:SteerGain   =     0.950000000     1.300000000     0.950000000     1.300000000
//TSI:SteerDamp   =     2.000000000     1.250000000     2.000000000     1.250000000
//TSI:SteerBias   =     9.000000000    25.000000000     9.000000000    25.000000000
//TSI:ExtraSpace  =    10.000000000    10.000000000    10.000000000    10.000000000
//TSI:CornerSetup =     0.800000000     0.800000000     0.800000000     0.800000000
//TSI:BrakeFactor =     0.000390000     0.000500000     0.000390000     0.000500000
//TSI:AccelFactor =     0.001000000     0.001000000     0.001000000     0.001000000
//TSI:STEFactor   =     3.610000000     3.610000000     3.610000000     3.610000000
//TSI:STEFactor2  =     2.200000000     2.200000000     2.200000000     2.200000000
//TSI:
//TSI:SegmentList   =             0               1               2               3  
//TSI:TweakMode     = Segment
//TSI:TweakNLaps    = 50
//TSI:TweakSStep    = 0.04
//TSI:TweakEStep    = 0.009
//TSI:ParameterList = BrakeFactor AccelFactor STEFactor STEFactor2 CornerSpeed
//TSI:
//TSI:/*
//TSI:
//TSI:! ZandVort.trk
//TSI:
//TSI:CourseNSeg    = 30
//TSI:CourseWidth   = 131.25
//TSI:CourseRadii   =      0 1312.34      0 426.51     0 -164.04     0 264.0      0 656.17      0 328.08      0 -492.13      0 492.13      0 574.15      0 -492.13      0 656.17      0 -65.62      0 246.06      0 -131.23     0  215.0
//TSI:CourseLengths = 2484.0     .44 492.13   1.57 710.0    1.22 98.43  1.44 656.17    .61 721.78    .79 328.08     .79 557.74   2.27 557.74    .44 131.23     .52 492.13    .35 328.08   3.14 164.04   1.31 328.08      .4 264.0 3.1416
//TSI:        
//TSI:CornerSpeed =     5.980191060     5.980191060     6.532856398     6.411724800     6.165120000     6.668193792     5.579386176     6.600178215     6.409788459     6.344408617     8.103271684     6.102235776     5.700000000     6.534829916     5.693251200     5.809440000     6.592363604     5.809440000  
//TSI:SteerGain   =     0.814148667     0.899619433     0.832000000     0.865280000     0.815360000     0.823264899     0.800000000     0.823264899     0.856454144     0.799052800     0.865018685     0.800000000     0.815360000     0.799052800     0.908615627     0.783071744     0.832000000     0.832000000  
//TSI:SteerDamp   =     1.384079308     1.513325537     1.529382400     1.497020179     1.605978121     1.455560288     1.384079308     1.468818857     1.483507046     1.470115891     1.441148800     1.414000000     1.412325824     1.483507046     1.414000000     1.414000000     1.498794752     1.455560288  
//TSI:SteerBias   =     8.400000000     8.893189182     8.380120576     8.400000000     8.400000000     8.304475845     8.400000000     8.561280000     8.390054400     8.400000000     8.736000000     8.400000000     8.646892800     8.400000000     9.630275666     8.057808246     8.400000000     9.257083964  
//TSI:ExtraSpace  =    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000    10.000000000  
//TSI:CornerSetup =     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000     0.800000000  
//TSI:BrakeFactor =     0.000352517     0.000366617     0.000349440     0.000363418     0.000335501     0.000336000     0.000359285     0.000338958     0.000336000     0.000338958     0.000336000     0.000362987     0.000336000     0.000335205     0.000335602     0.000362987     0.000336000     0.000336000  
//TSI:AccelFactor =     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000     0.000800000  
//TSI:STEFactor   =     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000     3.000000000  
//TSI:STEFactor2  =     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000     1.500000000  
//TSI:
//TSI:TweakMode     = Segment
//TSI:TweakNLaps    = 25
//TSI:TweakSStep    = 0.04
//TSI:TweakEStep    = 0.009
//TSI:SegmentList   = 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29
//TSI:ParameterList = BrakeFactor CornerSpeed STEFactor STEFactor2 
//TSI:
//TSI:/*


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

// Used to be the file config.h

#ifndef CONFIG_H
#define CONFIG_H 1

#include <dos.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>


//
// A single item in an array that describes possible items in a configuration
// file.
//
typedef struct _config_template
{ char   *name;       // Name to search config file for.      
  short   count;      // Count of parameters to match.        
  char    size;       // sizeof() each item, applies to arrays (count > 1)
  char    allocate;   // boolean - 1 indicates need to malloc space for string.
  char   *in_format;  // scanf()  format string for parameters.
  void   *params;     // Pointer to parameters for storage.  
} CONFIG_TEMPLATE;


//
// Just a number and an associated text string
// Used to evaluate text strings to the appropriate numbers.
//
typedef struct _ttn
{  int  N;
  char *text;
} ttn_t;


 int   ReadConfigFilePiece( FILE *fp, CONFIG_TEMPLATE list[] , ttn_t *ttn );
void   InterpretConfigFileLine( char *buf, CONFIG_TEMPLATE list[] , ttn_t *ttn );
void   InterpretConfigFileParameters( char *cp, CONFIG_TEMPLATE *tp , ttn_t *ttn );
 int   text_to_number( char *text , ttn_t *ttn );
char  *number_to_text( int n , ttn_t *ttn );
 int   str_cmptrim(const char *string1, const char *string2);
char **array_of_words( char *string );
char  *line_starts_with( char *line, char *tag );

#endif


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

// Used to be the file config.cpp

// config.cpp - A fairly generic configuration file reader
// Ported into RARS by Mike Inman, April 1995

// #include "config.h"

//
// Entry point for piece readers (which read up to a /*, then return)
//
int ReadConfigFilePiece( FILE *fp, CONFIG_TEMPLATE list[] , ttn_t *ttn )
{ char  string[1200];
  char *buf = string;
  char *buf2;
  
  while ( fgets( buf, 1199 , fp ) != NULL )
    { buf2 = line_starts_with( buf , "//TSI:" );  // All lines pertaining to Track Setup Info
                                                  // Begin with a //TSI:
      if ( buf2 != NULL )
        { if ( (*buf2 == '/') && (*(buf2+1) == '*' ) )  // found a /*... we're done reading this piece
            return 1;
          InterpretConfigFileLine( buf2 , list , ttn );
        }
    }

  return 0;
}


//
// Takes one line of text from a config file and interprets it according to
// the CONFIG_TEMPLATE.
//
void InterpretConfigFileLine( char *buf, CONFIG_TEMPLATE list[] , ttn_t *ttn )
{
  CONFIG_TEMPLATE *tp;
  char            *cp, *np;


  np = buf; 
  while ( isspace(*np) && *np != '\0' )
    np++;                                            // skip leading blanks
  if ( *np == '\0' )                                  
    return;                                          // empty line, so return

  cp = np;
  while ( *cp != '=' && *cp != '\0' )
    cp++;                                            // get line name until =
  if ( *cp ==  '\0' )                                   
    return;                                          // if no = then return
  *(cp++) = '\0';                                    // np = name;cp = param
  

  for ( tp = list; tp->name != NULL; tp++ )          // search in list for name
    { if ( str_cmptrim( tp->name, np ) )             // if name found
        { while ( isspace(*cp) && *cp != '\0' )
            cp++;                                    // skip blanks after =
          if ( *cp != '\0' )                         // if not empty read params
            InterpretConfigFileParameters( cp , tp , ttn );
          return;                                    // either way, this one is done
        }
    }
  return;                                            // name not found
}


//
// Given a list of parameters and the CONFIG_TEMPLATE entry describing what
// to do with them, do it.
//
void InterpretConfigFileParameters( char *cp, CONFIG_TEMPLATE *tp , ttn_t *ttn )
{ void  *temp_p;
  char   string[120]; 
  char **array;
   int  *n_list;
   int   i;

  if ( strcmp( tp->in_format , "TN" ) == 0 )   // Is this the special Text to number formatted line?
    { array = array_of_words( cp );
      n_list = (int *)tp->params;              // These are always arrays of integers
      i = 0;
      while ( array[i] != NULL )
        { *n_list = text_to_number( array[i] , ttn );
          n_list++;
          i++;
        }
      n_list++;
      *n_list = -1; // End of list is marked by a -1
      return;
    }

  if ( tp->allocate ) // Only applies to strings, allocates memory to contain whatever is on the line (up to 120 characters)
    { sscanf( cp, tp->in_format, string );
      if ( (temp_p = malloc( strlen( string ) + 2 )) == NULL )
        { printf( "Error during malloc(), not enough free memory?\n" );
          exit( 1 );
        }
      memcpy( tp->params , &temp_p , sizeof( temp_p ) ); 
      strcpy( (char *)temp_p , string );
      return;
    }

  if ( tp->count == 1 ) // Single item, handle simply
    { sscanf( cp, tp->in_format, tp->params );
      return;
    }

  // Read each parameter in succession
  i = 0;
  while (( *cp != '\0' ) && ( i < tp->count ))
    { sscanf( cp, tp->in_format, (void *)((char *)tp->params + (i * tp->size)) );  // Read the first item in line

      while ( !isspace(*cp) && *cp != '\0' )     // Read to end of first item
        cp++;                                    
      while ( isspace(*cp) && *cp != '\0' )      // Skip blanks
        cp++;
      i++;                                       // Do the next item in the array
    }
}


//
// Given a string, attempt to match it to the ttnumber list
//   and return its value.
// If there is no match, return -1.
//
int text_to_number( char *text , ttn_t *ttn )
{ int i = 0;
  while (( ttn[i].text != NULL ) && ( ttn[i].N != -1 ))
    { if ( strcmp( text , ttn[i].text ) == 0 )
        return ttn[i].N;
      i++;
    }
  return -1;  // Bad entry constitutes an automatic End of List marker
}

//
// Find N in the list and return the corresponding string
// If there is no match, return "".
//
char *number_to_text( int n , ttn_t *ttn )
{ int i = 0;
  while (( ttn[i].text != NULL ) && ( ttn[i].N != -1 ))
    { if ( n == ttn[i].N )
        return ttn[i].text;
      i++;
    }
  return ""; 
}


//
// Some handy custom string manipulation functions
//

//
//  Compares string1 with string2 ignoring any leading or        
//  trailing blanks in either string                             
//  Returns 0 : if both strings are different                            
//          1 : if both strings are the same                             
//                                                                      
int  str_cmptrim(const char *string1, const char *string2)
{
  unsigned int i,j,k,l;

  int ok;

  if( string1 == NULL && string2 == NULL) return 1;
  if( string1 == NULL || string2 == NULL) return 0;

  if( string1[0] == '\0' && string2[0] == '\0') return 1;
  if( string1[0] == '\0' || string2[0] == '\0') return 0;

  k = l = 0;
  for (i=0; string1[i] != '\0'; i++)
    if ( string1[i] != ' ' && string1[i] != '\t' )
    {
      k = i;
      break;
    }

  for (i=0; string2[i] != '\0'; i++)
    if ( string2[i] != ' ' && string2[i] != '\t' )
    {
      l = i;
      break;
    }

  for (i=strlen(string1)-1; i > k; i--)
    if ( string1[i] != ' ' && string1[i] != '\t' ) break;

  for (j=strlen(string2)-1; j > k; j--)
    if ( string2[j] != ' ' && string2[j] != '\t' ) break;

  for(; string1[k] == string2[l] && k < i && l < j; k++,l++);

  ok =( ((string1[k] == '\t' || string1[k] == ' ') && (string2[l] == '\t' || string2[l] == ' ')) || (string1[k] == string2[l]) );

  return ( (k == i && l == j && ok   )?1:0 );
}



//
// Takes a list of blank separated words and converts it to an array of
// strings where each word is now a null terminated string.  Uses the same
// space that the original list occupied, replacing blanks with nulls.
// The array of pointers is placed after the end of the original string,
// so plenty of extra space should be provided for this.
//
// Considers either a null or /r or /n to be the end of the original list.
//
char **array_of_words( char *string )
{  int   i = 0;
   int   j = 0;
   int   k = 0;
  char **array;

  while (( string[i] != '\0' ) &&   // Find the end of the list
         ( string[i] != '\r' ) &&
         ( string[i] != '\n' ))
    i++;

  string[i] = ' ';                  // Replace the end of list with a space (so all words will end in a space)
  while ( string[i-1] == ' ' )      // Eliminate consisderation of any trailing blanks 
    i--;
  array = (char **)&(string[i+1]);  // Locate the array after the list

  while ( j < i )
    { while ( string[j] == ' ' )
        j++;                        // Skip leading or extra blanks

      if ( j < i )
        { array[k++] = (char *)&(string[j]);  // Array element points to first non-blank character

          while( string[j] != ' ' )
            j++;                    // Scan to next blank
          string[j] = '\0';         // Convert the blank to a NULL, terminating the array-word-string
          j++;
        }
    }
  array[k] = NULL;                  // NULL indicates the end of the array of words

  return array;
}

//
// Line Starts With...
// Sees if the line starts with the given string (tag), if it does, 
// returns the part of the line that follows the given string.
// Otherwise, returns NULL, indicating no match.
//
char *line_starts_with( char *line, char *tag )
{ int i = 0;

  while ( tag[i] != NULL )
    { if ( line[i] != tag[i] )
        return NULL;
      i++;
    }
  return &(line[i]);
}


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

// The "real" indretti code begins here

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <conio.h>
#include <stdio.h>
#include <malloc.h>
#include "track.h"
#include "car.h"

#ifdef WATCOM
#include <graph.h>
#endif 


extern    char *glob_name;       // The name string, below, will be copied here

segment *trackout = get_track_description().rgtwall;
segment *trackin = get_track_description().lftwall;
int  NSEG = get_track_description().NSEG;
double  width = get_track_description().width;


static     int  seg = 0; // Keeps track of what segment car is in, would prefer to
                         // access this information from the situation, but that is
                         // not possible for the 4-30-95 races.

// Maximum number of track segments
#define MAXSEG          64
// Maximum number of driver parameters
#define NPARM           15

//
// These are "default" values that should be good for any track.
//
static double CORNER_FACT        = 5.7;     // The "corner speed constant"
static double STEER_GAIN         = 0.8;     // servo gain, for staying in "lane"
static double STEER_DAMP         = 1.414;   // damping factor to prevent oscillation
static double STEER_BIAS         = 8.4;     // bias factor for steering through corners
static double EXTRA_SPACE        = 10.0;    // how far away from the walls to target the car
static double CORNER_SETUP       = 0.8;     // where to setup on the track for a corner
static double BRAKE_FACT         = 0.000336;// The "braking ability constant"
static double ACCEL_FACT         = 0.0008;  // The "acceleration ability constant"
static double STE_FACT           = 3.0;     // The "see the end of a curve constant"
static double STE_FACT2          = 1.5;     // The other "see the end of a curve constant"

// Array pointers, for arrays of the above variables (to specialize them segment by segment)
static double *aCORNER_FACT  ;
static double *aSTEER_GAIN   ;
static double *aSTEER_DAMP   ;
static double *aSTEER_BIAS   ;
static double *aEXTRA_SPACE  ;
static double *aCORNER_SETUP ;
static double *aBRAKE_FACT   ;
static double *aACCEL_FACT   ;
static double *aSTE_FACT     ;
static double *aSTE_FACT2    ;

//
// An array of pointers to the arrays of factors
// Just an easier way to get ahold of all the factors with one variable
//
// NOTE: The order of the variables in this array must correspond to the
// numbers and names of the factors in the ttnumber array.
//
// Also note: These "Initalizing values" will be overwritten when space
// is allocated in ConfigureIndretti()
//
static double *factors[] =
{aCORNER_FACT       
,aSTEER_GAIN        
,aSTEER_DAMP        
,aSTEER_BIAS         
,aEXTRA_SPACE       
,aCORNER_SETUP      
,aBRAKE_FACT        
,aACCEL_FACT        
,aSTE_FACT          
,aSTE_FACT2         
, NULL
}; 

//
// An array of pointers to the default values for the factors.
// 
// Note: the order of this array must match the order of factors[]
//
static double *def_fact[] =
{&CORNER_FACT       
,&STEER_GAIN        
,&STEER_DAMP        
,&STEER_BIAS         
,&EXTRA_SPACE       
,&CORNER_SETUP      
,&BRAKE_FACT        
,&ACCEL_FACT        
,&STE_FACT          
,&STE_FACT2         
, NULL
}; 


#define TWEAK_COURSE    99
#define TWEAK_SEGMENTS  98
#define TWEAK_OFF       97

//
// More Config file variables
//
static     int  TweakMode  = TWEAK_OFF;      
static     int  TweakNLaps = 25;      
static  double  TweakSStep = 0.04;      
static  double  TweakEStep = 0.009;      
static     int  cNSEG      = 0;          
static  double  cwidth     = 0.0;
static  double *Radii;
static  double *Lengths;
static     int *Segments;
static     int *Parameters;

//
// This is a list of Text to number conversions.
// CONFIG_TEMPLATE items that have TN as a format will
// use this list to convert the text that follows them into
// numbers (either a single int, or an array of ints)
//
static ttn_t indretti_ttn[] = 
{{ TWEAK_OFF     , "Off"            }
,{ TWEAK_COURSE  , "Course"         }
,{ TWEAK_SEGMENTS, "Segment"        }
,{             0 , "CornerSpeed"    }
,{             1 , "SteerGain"      }
,{             2 , "SteerDamp"      }
,{             3 , "SteerBias"      }
,{             4 , "ExtraSpace"     }
,{             5 , "CornerSetup"    }
,{             6 , "BrakeFactor"    }
,{             7 , "AccelFactor"    }
,{             8 , "STEFactor"      }
,{             9 , "STEFactor2"     }
,{            -1 ,  NULL            }
};


//
// The config file template for indretti.tsi (Track Setup Info)
// 
static CONFIG_TEMPLATE indretti_config[] =
{{ "CourseRadii"   ,MAXSEG,sizeof(double),0,"%lf" , Radii         }
,{ "CourseLengths" ,MAXSEG,sizeof(double),0,"%lf" , Lengths       }
,{ "SegmentList"   ,MAXSEG,   sizeof(int),0,"%d"  , Segments      }
,{ "ParameterList" , NPARM,   sizeof(int),0,"TN"  , Parameters    }
,{ "CornerSpeed"   ,MAXSEG,sizeof(double),0,"%lf" , aCORNER_FACT  }
,{ "SteerGain"     ,MAXSEG,sizeof(double),0,"%lf" , aSTEER_GAIN   }
,{ "SteerDamp"     ,MAXSEG,sizeof(double),0,"%lf" , aSTEER_DAMP   }
,{ "SteerBias"     ,MAXSEG,sizeof(double),0,"%lf" , aSTEER_BIAS   }
,{ "ExtraSpace"    ,MAXSEG,sizeof(double),0,"%lf" , aEXTRA_SPACE  }
,{ "CornerSetup"   ,MAXSEG,sizeof(double),0,"%lf" , aCORNER_SETUP }
,{ "BrakeFactor"   ,MAXSEG,sizeof(double),0,"%lf" , aBRAKE_FACT   }
,{ "AccelFactor"   ,MAXSEG,sizeof(double),0,"%lf" , aACCEL_FACT   }
,{ "STEFactor"     ,MAXSEG,sizeof(double),0,"%lf" , aSTE_FACT     }
,{ "STEFactor2"    ,MAXSEG,sizeof(double),0,"%lf" , aSTE_FACT2    }
,{ "TweakMode"     ,     1,   sizeof(int),0,"TN"  ,&TweakMode     }
,{ "TweakNLaps"    ,     1,             0,0,"%d"  ,&TweakNLaps    }
,{ "TweakSStep"    ,     1,             0,0,"%lf" ,&TweakSStep    }
,{ "TweakEStep"    ,     1,             0,0,"%lf" ,&TweakEStep    }
,{ "CourseNSeg"    ,     1,             0,0,"%d"  ,&cNSEG         }
,{ "CourseWidth"   ,     1,             0,0,"%lf" ,&cwidth        }
,{ NULL        , -1 }
};



//
// MI 4-95
// Take the current configuration variables and write them into a
// .twk file.  This means updates to the .tsi file must be done
// manually.  For now, I prefer things this way.
//
void WriteIndrettiConfig( void )
{    int  i,j;
    FILE *fp;
  double *flist;

  fp = fopen( "indretti.twk" , "w" );
  if ( fp == NULL )
    return;
  
  // Output the control factor matrix
  i = 0;
  while (( factors[i] != NULL ) && ( i < NPARM ))
    { fprintf( fp , "//TSI:%s =" , number_to_text( i , indretti_ttn ) );
      flist = factors[i];
      for ( j = 0 ; j < NSEG ; j++ )
        fprintf( fp , " %15.9lf" , flist[j] );
      fprintf( fp , "\n" );
      i++;
    }

  // Other stuff might follow, but this gets the job done

  fclose( fp );
}  
        

//
// Tweaking routines.  MI 3-95  Just dumping all the control factors into 
// this routine and letting it run forever will NOT get you the fastest setup
// for your driver, but it can help reduce the work in finding a good setup.
//

// State variables
#define TWK_START   1
#define TWK_SCAN    2


//
// MI 3-95
// tweak() should be called at the end of each lap, and passed a time
// for that lap.  You probably want to run solo, or if you are working
// on passing algorithms, you might want to run with traffic.
// My tweak caller only works while running solo.
//      
void tweak( double lap_time )
{
  static    int state       = TWK_START;
  static    int lap_count   = 0;
  static    int wp          = 0; // Parameter that is currently being worked on
  static    int ws          = 0; // Segment that is currently being worked on 
  static double step        ;
  static double start_time  ;
  static double start_value ;
  static double last_time   = 0.0;
  static double this_time   = 0.0;
  static double this_value  = 0.0;
  static double very_best   = 999999.0;
  static double best_time   = 999999.0;
  static double best_value  ;
  static double *flist;
  static   char string[80];
  static    int i;
  static   FILE *fp;
  static    int improved  = 0;
  static   long laps_done = 0;

  laps_done++;

  if (( TweakMode != TWEAK_COURSE   ) &&
      ( TweakMode != TWEAK_SEGMENTS ))
    return;

  this_time += lap_time;
  if ( ++lap_count < TweakNLaps )
    return;

  if ( TweakMode == TWEAK_COURSE )
    { Segments[0] = 0;
      ws = 0;
    }
  flist = factors[Parameters[wp]];
  this_value = flist[Segments[ws]];

  switch ( state )
    { case TWK_START:
        flist = factors[Parameters[wp]];
        start_value  = flist[Segments[ws]];
        start_time   = this_time;
        best_time    = this_time;
        best_value   = start_value;
        step         = TweakSStep;
        if ( TweakMode == TWEAK_SEGMENTS )
          { flist = factors[Parameters[wp]];
            flist[Segments[ws]] *= ( 1.0 + step );
          }
        if ( TweakMode == TWEAK_COURSE )
          for ( i = 0 ; i < NSEG ; i++ )
            { flist = factors[Parameters[wp]];
              flist[i] *= ( 1.0 + step );
            }
        state = TWK_SCAN;
        break;

      case TWK_SCAN:
        if ( this_time <= last_time )
          { // The change worked, keep going with it
            if ( this_time < best_time )
              { best_time    = this_time;
                flist = factors[Parameters[wp]];
                best_value   = flist[Segments[ws]];
                improved     = 1;

                if ( best_time < very_best )
                  { very_best = best_time;
                    WriteIndrettiConfig();
                  }
              }
            if ( TweakMode == TWEAK_SEGMENTS )
              { flist = factors[Parameters[wp]];
                flist[Segments[ws]] *= ( 1.0 + step );
              }
            if ( TweakMode == TWEAK_COURSE )
              for ( i = 0 ; i < NSEG ; i++ )
                { flist = factors[Parameters[wp]];
                  flist[i] *= ( 1.0 + step );
                }
          }
         else
          { // Improvement has run-out , back up & take smaller steps
            step *= -0.5;
            if ( TweakMode == TWEAK_SEGMENTS )
              { flist = factors[Parameters[wp]];
                flist[Segments[ws]] *= ( 1.0 + step );
              }
            if ( TweakMode == TWEAK_COURSE )
              for ( i = 0 ; i < NSEG ; i++ )
                { flist = factors[Parameters[wp]];
                  flist[i] *= ( 1.0 + step );
                }

            if (( step < TweakEStep ) && ( -step < TweakEStep ))
              { // Done tweaking this one, go to the next
                if ( TweakMode == TWEAK_SEGMENTS )
                  { flist = factors[Parameters[wp]];
                    flist[Segments[ws]] = best_value;
                  }
                if ( TweakMode == TWEAK_COURSE )
                  for ( i = 0 ; i < NSEG ; i++ )
                    { flist = factors[Parameters[wp]];
                      flist[i] = best_value;
                    }


                if ( Parameters[ ++wp ] < 0  )
                  { wp = 0;
                    if ( TweakMode == TWEAK_SEGMENTS )
                      { if ( Segments[ ++ws ] < 0 )
                          ws = 0;
                      }
                     else
                      { if ( improved )
                          improved = 0;
                         else
                          { printf( "No improvement found, quitting.\n" );
                            exit( 0 );
                          }
                      } 
                  }
                state = TWK_START;
              }
             else
              state = TWK_SCAN;
          }
        break;

      default:
        state = TWK_START;
        break;
    }


// System dependant code
#ifdef WATCOM
  _settextposition( 1 , 1 );
  sprintf( string , "Start Time %7.2lf Start Value %12.9lf\n", start_time/TweakNLaps, start_value ); _outtext( string );
  sprintf( string , " Best Time %7.2lf  Best Value %12.9lf\n",  best_time/TweakNLaps,  best_value ); _outtext( string );
  sprintf( string , " Last Time %7.2lf  Last Value %12.9lf\n",  this_time/TweakNLaps,  this_value ); _outtext( string );
  sprintf( string , " Step Size %7.4lf Var %s Seg %d Laps %ld  \n", step, number_to_text( Parameters[wp] , indretti_ttn ), Segments[ws] , laps_done ); _outtext( string );
#endif

  lap_count = 0;         
  last_time = this_time; // For future reference                                           
  this_time = 0.0;       // Reset accumulator
}


//
// MI 4-95
// Similar - are the two numbers within 2% of each-other?
// I use this comparison because floating point numbers
// have been known to vary slightly.  Also, minor tuning to
// a track should not require the driver to re-learn it.
//
static int similar( double a , double b )
{ double d;

  if ( a == b )
    return 1;

  if (( a == 0.0 ) || ( b == 0.0 ))
    return 0;

  d = (a-b)/a;
  if (( d > -0.02 ) && ( d < 0.02 ))
    return 1;

  return 0;
}


//
// MI 4-95
// Track segment compare function.
// Compare the radius, length and width passed to the corresponding
// segment of the current track.  If they agree within +-2%, return
// a 1 (match) otherwise return a 0 (no match)
//
static int ts_compare( int i , double r , double l , double w )
{ 
  double rad = trackout[i].radius - trackin[i].radius;
  if ( rad < 0.0 )
    rad = -rad;

  if ( similar( r , trackout[i].radius ) && 
       similar( l , trackout[i].length ) )
    { if ( r != 0 )
        { if ( similar( w , rad ) )
            return 1;
           else
            return 0;
        }
       else
        return 1;
    }
  return 0;
}


//
// MI 4-95
// Compare the currently read track to the one in use.
//
int MatchTrack( void )
{ int match, i;

  if ( cNSEG != NSEG )
    return 0;

  match = 1;
  i = 0;
  while ( i < NSEG )
    { if ( !ts_compare( i , Radii[i] , Lengths[i] , cwidth ) )
        match = 0;
      i++;
    }

  return match;
}


//
// MI 4-95
// Clear out the variables to be read from the config file
// so they don't contain "residual" values from previous tracks
//
static void ClearVars( void )
{    int  i,j;
  double *flist;

  TweakMode = TWEAK_OFF;

  for ( i = 0 ; i < MAXSEG ; i++ )
    Segments[i] = -1;
  for ( i = 0 ; i <  NPARM ; i++ )
    Parameters[i] = -1;

  for ( i = 0 ; i < NSEG ; i++ )
    { j = 0;
      while ( def_fact[j] != NULL )
        { flist = factors[j];
          flist[i] = *(def_fact[j]);
          j++;
}   }   }


// 
// MI 4-95
// If there is a "indretti.cpp" file, try to match the current
// track to one found in the file and set the driver's parameters 
// accordingly.
//
static int read_track_file( void )
{ FILE *fp;
   int  done  = 0;
   int  match = 0;

  fp = fopen( "indretti.cpp" , "r" );
  if ( fp == NULL )
    return 0;  // Could not open file, so give up and use default values

  while ( !done )
    { ClearVars();
      if ( !ReadConfigFilePiece( fp, indretti_config, indretti_ttn ) ) 
        done = 1;
      if ( feof( fp ) ) 
        done = 1;
      if ( MatchTrack() )
        { done  = 1;
          match = 1;
        }
    }    
  fclose( fp );

  return match;
}


//
// MI 4-95
// Allocate memory for all these arrays.
// Read the .tsi file to see if we have a match, if so, use it.
// If not, paste over all the values with the default values.
//
void ConfigureIndretti( void )
{ Radii          = new double[MAXSEG]; 
  Lengths        = new double[MAXSEG]; 
  Segments       = new    int[MAXSEG]; 
  Parameters     = new    int[ NPARM]; 

  indretti_config[0].params = (void *)Radii     ; 
  indretti_config[1].params = (void *)Lengths   ; 
  indretti_config[2].params = (void *)Segments  ; 
  indretti_config[3].params = (void *)Parameters; 

  aCORNER_FACT   = factors[0] = new double[MAXSEG];
  aSTEER_GAIN    = factors[1] = new double[MAXSEG];
  aSTEER_DAMP    = factors[2] = new double[MAXSEG];
  aSTEER_BIAS    = factors[3] = new double[MAXSEG];
  aEXTRA_SPACE   = factors[4] = new double[MAXSEG];
  aCORNER_SETUP  = factors[5] = new double[MAXSEG];
  aBRAKE_FACT    = factors[6] = new double[MAXSEG];
  aACCEL_FACT    = factors[7] = new double[MAXSEG];
  aSTE_FACT      = factors[8] = new double[MAXSEG];
  aSTE_FACT2     = factors[9] = new double[MAXSEG];

  indretti_config[ 4].params = (void *)aCORNER_FACT ; 
  indretti_config[ 5].params = (void *)aSTEER_GAIN  ; 
  indretti_config[ 6].params = (void *)aSTEER_DAMP  ; 
  indretti_config[ 7].params = (void *)aSTEER_BIAS  ; 
  indretti_config[ 8].params = (void *)aEXTRA_SPACE ; 
  indretti_config[ 9].params = (void *)aCORNER_SETUP; 
  indretti_config[10].params = (void *)aBRAKE_FACT  ; 
  indretti_config[11].params = (void *)aACCEL_FACT  ; 
  indretti_config[12].params = (void *)aSTE_FACT    ; 
  indretti_config[13].params = (void *)aSTE_FACT2   ; 

  if ( !read_track_file() )
    ClearVars(); // Couldn't find this track in the file, so use the default values

}

//
// Sometimes it is helpful to see the current "state" on-screen
// Calling dump_indretti will display several variables
//
static int dump = 0;
void dump_indretti( void )
{ dump = 1;
}



//////////////////////
// Driving Routines //
//////////////////////



//
// Given a radius (in feet), compute the desired speed (in feet per second)
// MI 3-95
//
static double desired_speed( double rad )
{ double val;

  if ( rad < 0.0 )
    rad = -rad;

  if (( rad > 10000.0 ) || ( rad == 0.0 ))
    return 500.0;

  if ( rad < 25.0 )
    rad = 25.0;

  val = sqrt( rad ) * aCORNER_FACT[seg]; 
  if ( val > 500.0 ) val = 500.0;

  return val;
}


//
// Given two desired speeds, compute the time required to go from s1 to s2
// MI 3-95
//
static double exit_time( double s1, double s2 )
{ double diffsq = ( s1 - s2 ) * ( s1 - s2 );

  if ( s1 > s2 )
    return diffsq * aBRAKE_FACT[seg]; 
  return diffsq * aACCEL_FACT[seg];
}


//
// Given a radius, distance, and width, determine if the driver cannot
//   "see the end" of this turn and therefore must hold off on 
//   acceleration/braking for the next segment.
// MI 3-95
//
int cannot_see_end( double r ,double d , double w )
{ 
  if ( r < 0 )
    r = -r;
  
  // Straight or nearly straight
  if (( r > 10000.0 ) || ( r == 0.0 ))    return 0;
                                         
  // More than x degrees to go
  if ( d / r > aSTE_FACT2[seg] )                return 1;

  // Road is too narrow to "see out"
  if ( aSTE_FACT[seg] * d * d > w * w + 2*r*w ) return 1;

  return 0;
}


/////////////////////////////////////////////////////////////////////
// The Michael Indretti driver, by Mike Inman 3-95                 //
//                                                                 //
// Nothing too fancy here, the real power of this driver is in the //
// tweaking and track identification routines.                     //
/////////////////////////////////////////////////////////////////////

con_vec Indretti(situation& s)
{
  const     char name[]    = "Indretti"; // This is the robot driver's name
  static  double last_to_end = 999999.9; // To watch for crossings into new segments
  static  double last_rad    = 999999.9; // To watch for crossings into new segments
  static     int init_flag = 1;          // cleared by first call
  static     int relax = 1;              // cleared first time through
         con_vec result;                 // This is what is returned
          double alpha;                  // Working variable
          double dest;                   // Destination location on track
          double tte;                    // Time to end (this segment) 
          double tv;                     // Target Velocity
          double bias;                   // Used in steering
          double length;                 // Linearized length number
          double to_end;                 // Linearized to_end length
          double cur_rad;
 
  if ( s.starting )
    { ConfigureIndretti();               // Try to match up the current track to a known one
      seg = 0;
      relax = 1;
    }

  if (init_flag)                         // first time through, copy name
    { my_name_is(name);
      init_flag = 0;
      result.alpha = result.vc = 0;
      return result;
    }

  cur_rad = s.cur_rad;
  if ( cur_rad < 0.0 ) 
    cur_rad = -cur_rad;

  // Straighten the curves
  if ( s.cur_rad == 0.0 )
    { length = s.cur_len;
      to_end = s.to_end;
    }
   else
    { length = s.cur_len * cur_rad;   
      to_end = s.to_end * cur_rad;
    }

  // Keep track of the current segment
  if ( relax )
    relax = 0;  // Used to get jumpy at the start of races & skip one
   else
    { if (( to_end > last_to_end ) && ( s.cur_rad != last_rad ))
        if ( ++seg >= NSEG )
          seg = 0;
    }

  last_to_end = to_end;
  last_rad    = s.cur_rad;

  // Distance to destination
  if ( to_end > length/2 )
    { // In the 1st half of the segment
      if ( s.cur_rad == 0 )
        { if ( s.nex_rad > 0 )
            dest = (s.to_lft + s.to_rgt) * (1.0 - aCORNER_SETUP[seg]);
           else
            dest = (s.to_lft + s.to_rgt - CARWID ) * aCORNER_SETUP[seg];
        }
       else
        { if ( s.cur_rad > 0 )
            dest = aEXTRA_SPACE[seg];
           else
            dest = s.to_lft + s.to_rgt - aEXTRA_SPACE[seg];
        }
    }
   else
    { // In the second half of the segment
      if ( cannot_see_end( s.cur_rad , to_end , s.to_lft + s.to_rgt ) )
        { if ( s.cur_rad > 0 )
            dest = aEXTRA_SPACE[seg];
           else
            dest = s.to_lft + s.to_rgt - aEXTRA_SPACE[seg];
        }
       else
        { if ( s.nex_rad == 0 )
            { if ( s.after_rad > 0 )
                dest = (s.to_lft + s.to_rgt) * (1.0 - aCORNER_SETUP[seg]);
               else
                dest = (s.to_lft + s.to_rgt - CARWID ) * aCORNER_SETUP[seg];
            }
           else
            { if (( s.cur_rad > 0 ) && ( s.nex_rad > 0 ))
                dest = aEXTRA_SPACE[seg];
              else if (( s.cur_rad < 0 ) && ( s.nex_rad < 0 ))
                dest = s.to_lft + s.to_rgt - aEXTRA_SPACE[seg];
              else
                { if ( s.nex_rad > 0 )
                    dest = (s.to_lft + s.to_rgt) * (1.0 - aCORNER_SETUP[seg]);
                   else
                    dest = (s.to_lft + s.to_rgt - CARWID ) * aCORNER_SETUP[seg];
                }
            }
        }
    }

  // Time to end
  if ( s.v > 1.0 )
    tte  = to_end / s.v; 
   else
    tte  = 60.0;  // Override setting incase of slow speeds


  // Desired driving speeds:
  //   If the time to the end of the segment is greater than exit_time()
  //   (which is determined by the relative desired_speed()s for the segments)
  //   hold speed to the desired speed for this segment, otherwise, target
  //   speed is the target speed for the next segment, unless in the middle
  //   of a corner where the driver "cannot see the end", then hold to
  //   the desired speed for this segment.

  if (( tte > exit_time( s.v , desired_speed( s.nex_rad ) ) ) ||
      cannot_see_end( s.cur_rad , to_end , s.to_lft + s.to_rgt ) )
    tv = desired_speed( s.cur_rad );
   else
    tv = desired_speed( s.nex_rad );

  // Steering:

  if (s.cur_rad == 0.0)     // calculate a bias for alpha in a turn:
    bias = 0.0;
   else 
    { if (s.cur_rad > 0.0)
        { if ( s.to_lft > 1.0 )
            bias =  aSTEER_BIAS[seg] / desired_speed( s.cur_rad );
           else
            bias =  0.0;
        }
       else
        { if ( s.to_rgt > CARWID - 1.0 )
            bias = -aSTEER_BIAS[seg] / desired_speed( s.cur_rad );
           else
            bias = 0.0;
    }   }

  alpha  = bias + aSTEER_GAIN[seg] * ( s.to_lft - dest ) / (s.to_lft + s.to_rgt);
  alpha -= aSTEER_DAMP[seg] * s.vn / s.v;  // This is damping, to prevent oscillation

  result.vc    = tv;
  result.alpha = alpha;
  if(s.starting)
	result.fuel_amount = MAX_FUEL;
  result.request_pit = 0;
  if (s.stage != QUALIFYING && (s.damage > 25000 || s.fuel < 10)){
	 result.request_pit = 1;
	 result.repair_amount=s.damage;
	 result.fuel_amount = MAX_FUEL;
  }


// This is system dependant code
#ifdef WATCOM
  if ( dump )
    { _settextposition( 1 , 1 );
      sprintf( string , "  alpha %7.4lf,      vc %7.2lf\n", alpha    , tv );        _outtext( string );
      sprintf( string , "      v %7.2lf,      vn %7.2lf\n", s.v      , s.vn );      _outtext( string );
      sprintf( string , " to_lft %7.2lf,    dest %7.2lf\n", s.to_lft , dest );  _outtext( string );
      sprintf( string , "cur_rad %7.2lf, cur_len %7.2lf\n", s.cur_rad, length ); _outtext( string );
      sprintf( string , " cur_ds %7.2lf,  nex_ds %7.2lf\n", desired_speed( s.cur_rad ) , desired_speed( s.nex_rad ) ); _outtext( string );
      sprintf( string , "    tte %7.2lf, exit_tm %7.2lf\n", tte, exit_time( s.v  , desired_speed( s.nex_rad )) ); _outtext( string );
      sprintf( string , "   cnse %1d seg %ld to_end %7.2lf\n", cannot_see_end( s.cur_rad , to_end , s.to_lft + s.to_rgt ), seg, to_end ); _outtext( string );
             
      _settextposition( 15 , 1 );
      sprintf( string , "  bias %7.4lf    gain %7.4lf\n", bias, aSTEER_GAIN[seg] );        _outtext( string );
      sprintf( string , "  dest %7.4lf to_left %7.4lf to_right %7.4lf cs %7.4lf\n", dest, s.to_lft, s.to_rgt, aCORNER_SETUP[seg] );  _outtext( string );
      sprintf( string , "  s.vn %7.4lf     s.v %7.4lf\n", s.vn, s.v  );        _outtext( string );
    }
#endif                                                    


  return result;
}
