#include <gtest/gtest.h>

#include "bumperbot_firmware/serial_protocol.hpp"

using bumperbot_firmware::find_wheel_index;
using bumperbot_firmware::format_wheel_command;
using bumperbot_firmware::parse_wheel_message;
using bumperbot_firmware::take_last_line;

TEST(SerialProtocol, ParsesBothWheels)
{
  auto r = parse_wheel_message("rp05.20,ln03.10,");
  EXPECT_TRUE(r.has_right);
  EXPECT_TRUE(r.has_left);
  EXPECT_DOUBLE_EQ(r.right, 5.2);
  EXPECT_DOUBLE_EQ(r.left, -3.1);
  EXPECT_EQ(r.rejected, 0);
}

TEST(SerialProtocol, IgnoresLineEndingsAndOrder)
{
  auto r = parse_wheel_message("ln12.50,rp00.00,\r\n");
  EXPECT_DOUBLE_EQ(r.left, -12.5);
  EXPECT_DOUBLE_EQ(r.right, 0.0);
  EXPECT_EQ(r.rejected, 0);
}

TEST(SerialProtocol, CorruptedTokensDoNotThrow)
{
  for (const std::string bad : {"r", "rp", "x", "rp,", "rq05.0,", "rp05.2x,", "lnabc,", ",,,", "", "rpnan,", "\xff\xfe"})
  {
    EXPECT_NO_THROW({
      auto r = parse_wheel_message(bad);
      EXPECT_FALSE(r.has_right) << bad;
      EXPECT_FALSE(r.has_left) << bad;
    });
  }
  auto partial = parse_wheel_message("p05.20,ln03.10,");
  EXPECT_FALSE(partial.has_right);
  EXPECT_TRUE(partial.has_left);
  EXPECT_EQ(partial.rejected, 1);
}

TEST(SerialProtocol, FormatsCommandsLikeTheFirmwareExpects)
{
  EXPECT_EQ(format_wheel_command(5.2, -3.1), "rp05.20,ln03.10,");
  EXPECT_EQ(format_wheel_command(-12.346, 0.0), "rn12.35,lp00.00,");
}

TEST(SerialProtocol, FormatThenParseRoundTrip)
{
  auto r = parse_wheel_message(format_wheel_command(-4.25, 7.5));
  EXPECT_DOUBLE_EQ(r.right, -4.25);
  EXPECT_DOUBLE_EQ(r.left, 7.5);
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
    EXPECT_FALSE(r.has_right) << bad;
    EXPECT_EQ(r.rejected, 1) << bad;
  }
}

TEST(SerialProtocol, TakeLastLineKeepsTheUnfinishedTail)
{
  std::string buffer = "rp01.00,lp01.00,\r\nrp02.00,lp02.00,\r\nrp03";
  std::string line;
  ASSERT_TRUE(take_last_line(buffer, line));
  EXPECT_EQ(line, "rp02.00,lp02.00,\r");
  EXPECT_EQ(buffer, "rp03");
  EXPECT_FALSE(take_last_line(buffer, line));
  buffer += ".00,lp03.00,\r\n";
  ASSERT_TRUE(take_last_line(buffer, line));
  EXPECT_EQ(parse_wheel_message(line).right, 3.0);
  EXPECT_TRUE(buffer.empty());
}

TEST(SerialProtocol, TakeLastLineDropsGarbageWithoutNewline)
{
  std::string buffer(300, 'x');
  std::string line;
  EXPECT_FALSE(take_last_line(buffer, line));
  EXPECT_TRUE(buffer.empty());
  buffer = "\nrp01.00,lp01.00,\n";
  ASSERT_TRUE(take_last_line(buffer, line));
  EXPECT_EQ(line, "rp01.00,lp01.00,");
}
