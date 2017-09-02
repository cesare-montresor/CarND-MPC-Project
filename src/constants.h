//
//  constants.h
//  MPCController
//
//  Created by Cesare on 02/09/2017.
//
//

#ifndef Contants_h
#define Contants_h



const size_t N = 10;
const double dt = 0.15;

const unsigned latency_ms = 100;
const double ref_velocity = 100;
const double Lf = 2.67;

const double mph2mps = 0.44704; // Google says
const double latency_s = latency_ms/1000;
const double ref_velocity_mps = ref_velocity * mph2mps;


#endif /* Contants_h */
