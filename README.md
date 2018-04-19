# Overview
This repository contains all codes that perform veicle localization through particle filter.- main.cpp include all data receiving, processing, and output RMSE of x, y and theta, provided by udacity;- particle_filter.cpp include all codes edited by myself, it's core coded including all functions which perform particle filter.

## Particle filter Introduction
Particle filter or Sequencial Monte Carlo methods are a set of genetic Monte Carlo algrothium which used to solve filtering problem. It's first coined by Del Moral in 1996 in reference to mean filed interacting Monte Carlo used in fulid mechanics since the begging of 1960s. The terminology of 'sequencial Monte Carlo' was proposed by Cui and Chen in 1998.

Particle filter with a set of particles to represent posterior distribution of some stochastic process given noise and/or particle observation. the object is to compute posterior distribution of the states of some markov process, given some noise and particle observation. The state model can be non-linear distribution and the initial state and noise can take any form required. Particle filter is a well-estiamated methodology for generating samples from the required distribution without required assumption of state-space model or state distribution. However Particle filter do not perform well when applied to high-dimenstional systems, beacuse efferency is expontial order.

## Particle filter process
![image](https://github.com/Genzaiwuxian/udacity-term2-p3/blob/master/figure/particle%20filter%20process.octet-stream)

- initializationdetermine particle numbers, set particles around initial GPS localization as normal gaussian distribution.

- predictioneach particle moves as montion model given noise distribution.

- data association
associate landmark with real measurement (lindar and radar results). In this project, using neasret neighbor method, that means measurement will bounded with nearest landmark by cauculating distance between each landmarks and measurements.
the neaset neighbor method advantage and disadvantage:
  Advantage: easy to code, simple method and perform well at most conditions;
  Disadvantage: perform not well at high density measurements; not precise measurement with large noise; not precise motion model; low cauculation effencity, computation complexity= m*n which m is number of particles, and n is landmarks number.
  
- update step
after data associations, calculating distancse between each particles localizatio and landmarks, and then providing weights to each particls by multivariate gaussian probability density
![image](https://github.com/Genzaiwuxian/udacity-term2-p3/blob/master/figure/multivariate%20gaussian%20probability%20density.PNG)

- resample
resampling important samples by resampling wheel:
after normalizing all particles weights, total weights = 1, make each ID partils weights be connected by next order ID+1 particle like a cycle;
randomly select one weights between 0 to 2 times of max_weight as beta;
at each time comparing corrent weights with beta, if larger, duplicate it and do the next loop; if less, deta decrease this particle weights and indicator++, loop until weights is larger than beta.

## Different particle numbers
set particle number=10, 50, 100, 150, 200 and see performances:

![image](https://github.com/Genzaiwuxian/udacity-term2-p3/blob/master/figure/differnt%20num%20comparsion.PNG)
- accuracy of x, y, yaw localization decrease as prticle numbers increase;
- system time increase;
- partcile numbers=10~50 can achieve acceptable accuracy and system time;

## Number of effenccy particle
as particle filter running, most particles will focuse on a narrow area after sampling each time, that can be called as loss of diversity. at the method of sequencial importance resampling, it will calculate Number of Effentive particle, and compared it with Number of threshold which defined by tuning to determine whether perform resampling process or not. this can decrease resmpaling process, so it can improve lack of diversity performace to some extent;




## Reflection
- particle numbers=150, simulator run out of time, no sure why this situation happen;
