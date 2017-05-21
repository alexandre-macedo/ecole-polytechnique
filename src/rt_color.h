#pragma once
#ifndef RT_COLOR_H
#define RT_COLOR_H
class RtColor
{
  public:
    RtColor();
    ~RtColor();
    RtColor(int r, int g, int b);
    int getR() const;
    int getG() const;
    int getB() const;

  private:
    int r_, g_, b_;
};

#endif /* RT_COLOR_H */