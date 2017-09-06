#!/usr/bin/env python

__author__ = "aivanovic"

def deadzone(value, lower_limit, upper_limit):

    if(value > upper_limit): return(value - upper_limit)
    elif(value < lower_limit): return(value - lower_limit)
    else: return 0.0


def filterPT1(previous_output, current_value, T, Ts, K):

    a = T / (T + Ts)
    b = K*Ts / (T + Ts)

    return (a*previous_output + b*current_value)