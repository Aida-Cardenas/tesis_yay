#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "bumperbot_firmware/serial_protocol.hpp"

using bumperbot_firmware::find_wheel_index;
using bumperbot_firmware::format_wheel_command;
using bumperbot_firmware::link_action;
using bumperbot_firmware::LinkAction;
using bumperbot_firmware::parse_wheel_message;
using bumperbot_firmware::ReadingResult;
using bumperbot_firmware::take_last_line;
using bumperbot_firmware::WheelIntegrator;

namespace
{
// A line as the sketch sends it: speeds in rad/s, the encoder counters and the
// Arduino's millis(). By default the Arduino's clock is the PC's.
std::string line(double right, double left, unsigned right_count, unsigned left_count, double now_s,
                 long arduino_ms = -1)
{
  auto speed = [](char wheel, double v) {
    return std::string(1, wheel) + (v < 0 ? "n" : "p") + std::to_string(std::abs(v));
  };
  const long ms = arduino_ms >= 0 ? arduino_ms : std::lround(now_s * 1000.0);
  return speed('r', right) + "," + speed('l', left) + ",rc" + std::to_string(right_count) +
         ",lc" + std::to_string(left_count) + ",t" + std::to_string(ms) + ",";
}

// Feeds a reading taken at now_s
ReadingResult feed(WheelIntegrator & w, double now_s, double right, double left, unsigned right_count,
                   unsigned left_count, long arduino_ms = -1)
{
  return w.on_reading(now_s, parse_wheel_message(line(right, left, right_count, left_count, now_s, arduino_ms)));
}

// 0.01 rad per count keeps the expected positions easy to read
constexpr double kRadPerCount = 0.01;
}  // namespace

TEST(SerialProtocol, ParsesBothWheels)
{
  auto r = parse_wheel_message("rp4.55,ln03.10,rc100,lc65535,t4294967295,");
  EXPECT_TRUE(r.complete());
  EXPECT_EQ(r.time_ms, 4294967295u);
  EXPECT_FALSE(r.right_fault || r.left_fault);
  EXPECT_DOUBLE_EQ(r.right.speed, 4.55);
  EXPECT_DOUBLE_EQ(r.left.speed, -3.1);
  EXPECT_EQ(r.right.count, 100);
  EXPECT_EQ(r.left.count, 65535);
  EXPECT_EQ(r.rejected, 0);
}

TEST(SerialProtocol, IgnoresLineEndingsAndOrder)
{
  auto r = parse_wheel_message("lc5,t0,ln12.50,rc7,rp00.00,\r\n");
  EXPECT_TRUE(r.complete());
  EXPECT_DOUBLE_EQ(r.left.speed, -12.5);
  EXPECT_DOUBLE_EQ(r.right.speed, 0.0);
  EXPECT_EQ(r.right.count, 7);
  EXPECT_EQ(r.rejected, 0);
}

TEST(SerialProtocol, CorruptedTokensDoNotThrow)
{
  for (const std::string bad : {"r", "rp", "x", "rp,", "rq05.0,", "rp05.2x,", "lnabc,", ",,,", "", "rpnan,",
                                "rc", "rc-1,", "\xff\xfe"})
  {
    EXPECT_NO_THROW({
      auto r = parse_wheel_message(bad);
      EXPECT_FALSE(r.right.has_speed || r.right.has_count) << bad;
      EXPECT_FALSE(r.left.has_speed || r.left.has_count) << bad;
    });
  }
  auto partial = parse_wheel_message("p05.20,ln03.10,");
  EXPECT_FALSE(partial.right.has_speed);
  EXPECT_TRUE(partial.left.has_speed);
  EXPECT_FALSE(partial.complete());
  EXPECT_EQ(partial.rejected, 1);
}

TEST(SerialProtocol, CountsMustBePlainIntegersUpTo65535)
{
  for (const std::string bad : {"rc-1,", "rc1.5,", "rc65536,", "rc123456,", "rcx,", "rc+5,"})
  {
    auto r = parse_wheel_message(bad);
    EXPECT_FALSE(r.right.has_count) << bad;
    EXPECT_EQ(r.rejected, 1) << bad;
  }
  EXPECT_EQ(parse_wheel_message("rc65535,").right.count, 65535);
  EXPECT_EQ(parse_wheel_message("rc0,").right.count, 0);
}

TEST(SerialProtocol, TimeAndFaultsAreParsed)
{
  auto r = parse_wheel_message("rp0.00,lp0.00,rc1,lc2,t123,fr,fl,");
  EXPECT_TRUE(r.complete());
  EXPECT_EQ(r.time_ms, 123u);
  EXPECT_TRUE(r.right_fault);
  EXPECT_TRUE(r.left_fault);
  EXPECT_FALSE(parse_wheel_message("rp0.00,lp0.00,rc1,lc2,").complete());
  for (const std::string bad : {"t,", "t-1,", "t4294967296,", "t1.5,", "tx,"})
  {
    auto bad_time = parse_wheel_message(bad);
    EXPECT_FALSE(bad_time.has_time) << bad;
    EXPECT_EQ(bad_time.rejected, 1) << bad;
  }
}

TEST(SerialProtocol, FormatsCommandsLikeTheFirmwareExpects)
{
  EXPECT_EQ(format_wheel_command(5.2, -3.1), "rp05.20,ln03.10,");
  EXPECT_EQ(format_wheel_command(-12.346, 0.0), "rn12.35,lp00.00,");
}

TEST(SerialProtocol, FormatThenParseRoundTrip)
{
  auto r = parse_wheel_message(format_wheel_command(-4.25, 7.5));
  EXPECT_DOUBLE_EQ(r.right.speed, -4.25);
  EXPECT_DOUBLE_EQ(r.left.speed, 7.5);
}

TEST(SerialProtocol, CommandIsAlwaysReadableByTheSketch)
{
  EXPECT_EQ(format_wheel_command(std::nan(""), 1.0), "rp00.00,lp01.00,");
  EXPECT_EQ(format_wheel_command(INFINITY, -INFINITY), "rp00.00,lp00.00,");
  EXPECT_EQ(format_wheel_command(150.0, -150.0), "rp99.99,ln99.99,");
}

TEST(SerialProtocol, CommandIsRoundedBeforeTheSignAndPadding)
{
  EXPECT_EQ(format_wheel_command(9.996, -9.996), "rp10.00,ln10.00,");
  EXPECT_EQ(format_wheel_command(9.994, -0.004), "rp09.99,lp00.00,");
  EXPECT_EQ(format_wheel_command(-0.006, 99.996), "rn00.01,lp99.99,");
}

TEST(SerialProtocol, WheelsAreFoundByNameNotByPosition)
{
  std::vector<std::string> urdf_order = {"wheel_left_joint", "wheel_right_joint"};
  EXPECT_EQ(find_wheel_index(urdf_order, "right"), 1);
  EXPECT_EQ(find_wheel_index(urdf_order, "left"), 0);
  std::vector<std::string> swapped = {"wheel_right_joint", "wheel_left_joint"};
  EXPECT_EQ(find_wheel_index(swapped, "right"), 0);
  EXPECT_EQ(find_wheel_index(swapped, "left"), 1);
  std::vector<std::string> other_names = {"left_wheel", "right_wheel"};
  EXPECT_EQ(find_wheel_index(other_names, "right"), 1);
  EXPECT_EQ(find_wheel_index({"a", "b"}, "right"), -1);
}

TEST(SerialProtocol, NumbersMustBePlainDecimals)
{
  for (const std::string bad : {"rp-5.00,", "rp0x1A,", "rp+3.0,", "rp1e3,", "rpnan,"})
  {
    auto r = parse_wheel_message(bad);
    EXPECT_FALSE(r.right.has_speed) << bad;
    EXPECT_EQ(r.rejected, 1) << bad;
  }
}

TEST(SerialProtocol, TakeLastLineKeepsTheUnfinishedTail)
{
  std::string buffer = "rp01.00,lp01.00,\r\nrp02.00,lp02.00,\r\nrp03";
  std::string text;
  ASSERT_TRUE(take_last_line(buffer, text));
  EXPECT_EQ(text, "rp02.00,lp02.00,\r");
  EXPECT_EQ(buffer, "rp03");
  EXPECT_FALSE(take_last_line(buffer, text));
  buffer += ".00,lp03.00,\r\n";
  ASSERT_TRUE(take_last_line(buffer, text));
  EXPECT_EQ(parse_wheel_message(text).right.speed, 3.0);
  EXPECT_TRUE(buffer.empty());
}

TEST(SerialProtocol, TakeLastLineDropsGarbageWithoutNewline)
{
  std::string buffer(300, 'x');
  std::string text;
  EXPECT_FALSE(take_last_line(buffer, text));
  EXPECT_TRUE(buffer.empty());
  buffer = "\nrp01.00,lp01.00,\n";
  ASSERT_TRUE(take_last_line(buffer, text));
  EXPECT_EQ(text, "rp01.00,lp01.00,");
}

TEST(WheelIntegrator, PositionComesFromTheCounters)
{
  WheelIntegrator w(0.3, kRadPerCount);
  feed(w, 0.0, 1.0, -1.0, 1000, 1000);
  EXPECT_DOUBLE_EQ(w.right_position(), 0.0);
  feed(w, 0.1, 1.0, -1.0, 1100, 900);
  EXPECT_NEAR(w.right_position(), 1.0, 1e-9);
  EXPECT_NEAR(w.left_position(), -1.0, 1e-9);
}

TEST(WheelIntegrator, MovesWithTheLastSpeedBetweenReadingsAndSnapsBack)
{
  WheelIntegrator w(0.3, kRadPerCount);
  feed(w, 0.0, 2.0, -1.0, 0, 0);
  double t = 0.0;
  for (int i = 0; i < 10; ++i)
  {
    t += 0.01;
    EXPECT_FALSE(w.advance(t, 0.01));
  }
  EXPECT_NEAR(w.right_position(), 0.2, 1e-9);
  EXPECT_NEAR(w.left_position(), -0.1, 1e-9);
  feed(w, t, 2.0, -1.0, 19, 65526);
  EXPECT_NEAR(w.right_position(), 0.19, 1e-9);
  EXPECT_NEAR(w.left_position(), -0.10, 1e-9);
}

TEST(WheelIntegrator, LostLinesDoNotLoseDistance)
{
  WheelIntegrator w(0.3, kRadPerCount);
  feed(w, 0.0, 1.0, 1.0, 0, 0);
  feed(w, 0.2, 1.0, 1.0, 200, 200);
  EXPECT_NEAR(w.right_position(), 2.0, 1e-9);
}

TEST(WheelIntegrator, CounterWrapsAround)
{
  WheelIntegrator w(0.3, kRadPerCount);
  feed(w, 0.0, 1.0, -1.0, 65530, 10);
  feed(w, 0.1, 1.0, -1.0, 10, 65530);
  EXPECT_NEAR(w.right_position(), 0.16, 1e-9);
  EXPECT_NEAR(w.left_position(), -0.16, 1e-9);
}

TEST(WheelIntegrator, StopsWhenTheArduinoGoesQuiet)
{
  WheelIntegrator w(0.3, kRadPerCount);
  feed(w, 0.0, 5.0, 5.0, 0, 0);
  double t = 0.0;
  bool stale = false;
  for (int i = 0; i < 150; ++i)
  {
    t += 0.01;
    stale = w.advance(t, 0.01);
  }
  EXPECT_TRUE(stale);
  EXPECT_DOUBLE_EQ(w.right_velocity(), 0.0);
  EXPECT_DOUBLE_EQ(w.left_velocity(), 0.0);
  EXPECT_DOUBLE_EQ(w.right_position(), 0.0);
  feed(w, t, 1.0, 1.0, 150, 150);
  EXPECT_FALSE(w.advance(t + 0.01, 0.01));
  EXPECT_DOUBLE_EQ(w.right_velocity(), 1.0);
  EXPECT_NEAR(w.right_position(), 1.51, 1e-9);
}

TEST(WheelIntegrator, GarbageDoesNotKeepTheDataAlive)
{
  WheelIntegrator w(0.3, kRadPerCount);
  feed(w, 0.0, 5.0, 5.0, 0, 0);
  for (int i = 1; i <= 100; ++i)
  {
    w.on_reading(i * 0.01, parse_wheel_message("xx,rq,"));
  }
  EXPECT_TRUE(w.advance(1.0, 0.01));
}

TEST(WheelIntegrator, NoMotionBeforeTheFirstReading)
{
  WheelIntegrator w(0.3, kRadPerCount);
  EXPECT_TRUE(w.advance(0.5, 0.01));
  EXPECT_DOUBLE_EQ(w.right_position(), 0.0);
}

TEST(WheelIntegrator, IncompleteReadingsDoNotKeepTheDataAlive)
{
  WheelIntegrator w(0.3, kRadPerCount);
  feed(w, 0.0, 5.0, 5.0, 0, 0);
  for (int i = 1; i <= 100; ++i)
  {
    w.on_reading(i * 0.01, parse_wheel_message("rp05.00,rc10,"));
    w.on_reading(i * 0.01, parse_wheel_message("rp05.00,lp05.00,"));
  }
  EXPECT_TRUE(w.advance(1.0, 0.01));
  EXPECT_DOUBLE_EQ(w.left_velocity(), 0.0);
  EXPECT_TRUE(w.has_reading());
}

TEST(WheelIntegrator, ForgetReadingKeepsThePositionAndTakesANewReference)
{
  WheelIntegrator w(0.3, kRadPerCount);
  feed(w, 0.0, 5.0, -2.0, 0, 0);
  feed(w, 0.1, 5.0, -2.0, 50, 65516);
  w.advance(0.15, 0.05);
  w.forget_reading();
  EXPECT_FALSE(w.has_reading());
  EXPECT_DOUBLE_EQ(w.right_velocity(), 0.0);
  EXPECT_NEAR(w.right_position(), 0.5, 1e-9);
  EXPECT_NEAR(w.left_position(), -0.2, 1e-9);
  EXPECT_TRUE(w.advance(0.2, 0.1));
  EXPECT_NEAR(w.right_position(), 0.5, 1e-9);
  // The Arduino restarted: its counters begin again at 0
  feed(w, 0.3, 1.0, 1.0, 0, 0);
  EXPECT_NEAR(w.right_position(), 0.5, 1e-9);
  feed(w, 0.4, 1.0, 1.0, 10, 10);
  EXPECT_NEAR(w.right_position(), 0.6, 1e-9);
  EXPECT_NEAR(w.left_position(), -0.1, 1e-9);
}

TEST(WheelIntegrator, ArduinoRestartWithoutAPortErrorKeepsThePosition)
{
  // 300 readings forward, then a reset (brown-out, button): 1.5 s of silence and
  // the counters and the clock start again from 0
  WheelIntegrator w(0.3, kRadPerCount);
  double t = 0.0;
  for (unsigned i = 0; i <= 300; ++i)
  {
    t = 0.1 * i;
    EXPECT_NE(feed(w, t, 5.0, 5.0, 50 * i, 50 * i), ReadingResult::kArduinoRestarted);
  }
  const double before = w.right_position();
  EXPECT_NEAR(before, 150.0, 1e-6);
  t += 1.5;
  EXPECT_TRUE(w.advance(t, 0.01));
  EXPECT_EQ(feed(w, t, 0.0, 0.0, 0, 0, 100), ReadingResult::kArduinoRestarted);
  EXPECT_NEAR(w.right_position(), before, 1e-9);
  EXPECT_NEAR(w.left_position(), before, 1e-9);
  EXPECT_EQ(feed(w, t + 0.1, 1.0, 1.0, 10, 10, 200), ReadingResult::kAccepted);
  EXPECT_NEAR(w.right_position(), before + 0.1, 1e-9);
  // A small net travel before the reset is not taken as motion either
  WheelIntegrator small(0.3, kRadPerCount);
  feed(small, 0.0, 0.0, 0.0, 1000, 1000, 5000);
  feed(small, 0.1, 1.0, 1.0, 1020, 1020, 5100);
  EXPECT_EQ(feed(small, 1.0, 0.0, 0.0, 3, 3, 120), ReadingResult::kArduinoRestarted);
  EXPECT_NEAR(small.right_position(), 0.2, 1e-9);
}

TEST(WheelIntegrator, BrakingDoesNotShowAReverseSpeed)
{
  // Braking at a constant 20 rad/s^2. diff_drive_controller takes the speed from
  // the position over its last 10 cycles (100 ms): it must never be negative
  // while the wheel slows down, also with one lost line, and without lost lines
  // the position stays close to the real one.
  auto travel = [](double t) {
      const double braking = std::min(std::max(0.0, t - 0.3), 0.75);
      return 15.0 * std::min(t, 0.3) + 15.0 * braking - 10.0 * braking * braking;
    };
  for (int lost = 0; lost <= 12; ++lost)
  {
    WheelIntegrator w(0.3, 0.001);
    std::vector<double> shown;
    for (int step = 1; step <= 150; ++step)
    {
      const double t = 0.01 * step;
      if (step % 10 == 0 && (step / 10 != lost || lost == 0))
      {
        const double average = (travel(t) - travel(t - 0.1)) / 0.1;
        const auto counts = static_cast<unsigned>(std::lround(travel(t) / 0.001));
        feed(w, t, average, 0.0, counts, 0);
      }
      else
      {
        w.advance(t, 0.01);
      }
      shown.push_back(w.right_position());
      if (step >= 20)
      {
        EXPECT_GE(shown[step - 1] - shown[step - 11], -1e-9) << "t = " << t << ", lost " << lost;
        if (lost == 0)
        {
          EXPECT_NEAR(w.right_position(), travel(t) - travel(0.1), 0.12) << "t = " << t;
        }
      }
    }
  }
}

// A short stop at an experiment speed: diff_drive ramps the command down at its
// limit and the wheel follows it 50 ms later. Returns the lowest speed diff_drive
// would see (position over its last 10 cycles) from the start of the stop on.
// `phase` shifts the stop against the Arduino's readings; `use_command` passes
// the command sent in each cycle to advance().
double lowest_stop_speed(double from, double decel, double phase, bool use_command)
{
  const double stop_at = 0.5 + phase;
  auto command = [&](double t) {return std::max(0.0, std::min(from, from - decel * (t - stop_at)));};
  auto speed = [&](double t) {return command(t - 0.05);};
  auto travel = [&](double t) {
      double x = 0.0;
      for (double u = 0.00025; u < t; u += 0.0005)
      {
        x += speed(u) * 0.0005;
      }
      return x;
    };
  WheelIntegrator w(0.3, 2.0 * M_PI / 770.0);
  std::vector<double> shown;
  double lowest = INFINITY;
  for (int step = 1; step <= 150; ++step)
  {
    const double t = 0.01 * step;
    if (step % 10 == 0)
    {
      const double average = std::round((travel(t) - travel(t - 0.1)) / 0.1 * 100.0) / 100.0;
      feed(w, t, average, 0.0, static_cast<unsigned>(std::lround(travel(t) / (2.0 * M_PI / 770.0))) % 65536, 0);
    }
    else if (use_command)
    {
      w.advance(t, 0.01, command(t - 0.01), 0.0);
    }
    else
    {
      w.advance(t, 0.01);
    }
    shown.push_back(w.right_position());
    if (step > 10 && t > stop_at)
    {
      lowest = std::min(lowest, (shown[step - 1] - shown[step - 11]) / 0.1);
    }
  }
  return lowest;
}

TEST(WheelIntegrator, ShortStopsDoNotShowAReverseSpeed)
{
  // 4.5 rad/s is a line at 0.15 m/s, 1.5 rad/s a turn in place at 0.6 rad/s;
  // 21 rad/s^2 is diff_drive's 0.7 m/s^2 and 34 rad/s^2 adds the turn limit
  for (const double from : {4.5, 1.5})
  {
    for (const double decel : {21.0, 34.0})
    {
      for (int phase = 0; phase < 10; ++phase)
      {
        EXPECT_GE(lowest_stop_speed(from, decel, 0.01 * phase, true), -1e-9)
          << from << " rad/s, " << decel << " rad/s^2, phase " << phase;
      }
    }
  }
}

// Accelerating at a constant rate up to a cruise speed, the way diff_drive's
// limits ramp the commands. Returns the lowest speed diff_drive would see
// (position over its last 10 cycles) once the wheel cruises. `lost` is the
// reading that never arrives (0: none).
double lowest_cruise_speed(double accel, double cruise, int lost)
{
  WheelIntegrator w(0.3, 0.001);
  const double ramp_end = 0.2 + cruise / accel;
  auto travel = [&](double t) {
      const double ramp = std::min(std::max(0.0, t - 0.2), cruise / accel);
      return 0.5 * accel * ramp * ramp + cruise * std::max(0.0, t - ramp_end);
    };
  std::vector<double> shown;
  double lowest = INFINITY;
  for (int step = 1; step <= 150; ++step)
  {
    const double t = 0.01 * step;
    if (step % 10 == 0 && step / 10 != lost)
    {
      const double average = (travel(t) - travel(t - 0.1)) / 0.1;
      feed(w, t, average, 0.0, static_cast<unsigned>(std::lround(travel(t) / 0.001)), 0);
    }
    else
    {
      w.advance(t, 0.01);
    }
    shown.push_back(w.right_position());
    if (step > 10 && t > ramp_end + 0.1)
    {
      lowest = std::min(lowest, (shown[step - 1] - shown[step - 11]) / 0.1);
    }
  }
  return lowest;
}

TEST(WheelIntegrator, ReachingCruiseSpeedDoesNotShowADip)
{
  // 21 rad/s^2 is diff_drive's 0.7 m/s^2; 34 rad/s^2 adds the turn limit
  for (const double accel : {21.0, 34.0})
  {
    for (const double cruise : {3.0, 4.5})
    {
      EXPECT_GE(lowest_cruise_speed(accel, cruise, 0), 0.9 * cruise) << accel << " -> " << cruise;
      for (int lost = 3; lost <= 7; ++lost)
      {
        EXPECT_GE(lowest_cruise_speed(accel, cruise, lost), 0.0) << accel << " -> " << cruise << ", lost " << lost;
      }
    }
  }
}

TEST(WheelIntegrator, PredictionNeverReversesTheWheel)
{
  WheelIntegrator w(0.3, kRadPerCount);
  feed(w, 0.0, 15.0, -15.0, 0, 0);
  feed(w, 0.1, 1.0, -1.0, 10, 65526);
  const double right = w.right_position();
  const double left = w.left_position();
  for (int i = 1; i <= 20; ++i)
  {
    w.advance(0.1 + 0.01 * i, 0.01);
    EXPECT_GE(w.right_position(), right);
    EXPECT_LE(w.left_position(), left);
  }
}

TEST(WheelIntegrator, TrendIsNotTrustedAfterLostLines)
{
  WheelIntegrator w(0.3, kRadPerCount);
  feed(w, 0.0, 0.0, 0.0, 0, 0);
  feed(w, 0.3, 3.0, 3.0, 30, 30);
  w.advance(0.31, 0.01);
  EXPECT_NEAR(w.right_position(), 0.33, 1e-9);
}

TEST(ArduinoLink, WriteFollowsHowLongTheArduinoHasBeenQuiet)
{
  // Nothing arrives at all: the second time equals the first
  EXPECT_EQ(link_action(false, 0.0, 0.0), LinkAction::kWait);
  EXPECT_EQ(link_action(false, 4.9, 4.9), LinkAction::kWait);
  EXPECT_EQ(link_action(false, 5.1, 5.1), LinkAction::kReconnect);
  EXPECT_EQ(link_action(true, 0.05, 0.05), LinkAction::kSend);
  EXPECT_EQ(link_action(true, 0.3, 0.3), LinkAction::kSend);
  EXPECT_EQ(link_action(true, 0.31, 0.31), LinkAction::kSendZero);
  EXPECT_EQ(link_action(true, 1.0, 1.0), LinkAction::kSendZero);
  EXPECT_EQ(link_action(true, 1.01, 1.01), LinkAction::kSilent);
  EXPECT_EQ(link_action(true, 5.0, 5.0), LinkAction::kSilent);
  EXPECT_EQ(link_action(true, 5.01, 5.01), LinkAction::kReconnect);
}

TEST(ArduinoLink, AnOldSketchIsNotResetOverAndOver)
{
  // Lines keep arriving but none is complete (no `t`): the Arduino is alive
  EXPECT_EQ(link_action(false, 30.0, 0.05), LinkAction::kWait);
  EXPECT_EQ(link_action(true, 30.0, 0.05), LinkAction::kSilent);
}
