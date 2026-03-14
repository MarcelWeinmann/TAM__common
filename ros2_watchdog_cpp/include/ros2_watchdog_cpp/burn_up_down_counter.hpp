// Copyright 2023 Simon Hoffmann

#pragma once
#include <algorithm>
//
template <typename T>
class BurnUpDownCounter
{
public:
  BurnUpDownCounter(const T burn_up_rate, const T burn_down_rate, T saturation)
  : d_up_(burn_up_rate), d_down_(burn_down_rate), sat_(saturation), do_saturate_(true)
  {
  }
  BurnUpDownCounter(const T burn_up_rate, const T burn_down_rate)
  : d_up_(burn_up_rate), d_down_(burn_down_rate), do_saturate_(false)
  {
  }
  BurnUpDownCounter() = default;
  void up() { state_ = saturate(state_ + d_up_); }
  void down() { state_ = saturate(state_ - d_down_); }
  T get() { return state_; }
  BurnUpDownCounter<T> operator++()
  {
    up();
    return *this;
  }
  BurnUpDownCounter<T> operator--()
  {
    down();
    return *this;
  }
  void reset() { state_ = 0; }

private:
  T saturate(const T in) { return do_saturate_ ? std::max(0, std::min(in, sat_)) : in; }
  T d_up_, d_down_, state_, sat_;
  bool do_saturate_;
};
