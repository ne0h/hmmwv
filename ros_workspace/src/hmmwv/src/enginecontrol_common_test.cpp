#include <cppunit/extensions/HelperMacros.h>

#include <cstdio>

#include "enginecontrol_common.hpp"

struct test {
	struct cmd in;
	uint8_t out[2];
};

class EngineControlCommonTest : public CppUnit::TestFixture {
	CPPUNIT_TEST_SUITE(EngineControlCommonTest);
	CPPUNIT_TEST(testMarshalAndUnmarshal);
	CPPUNIT_TEST_SUITE_END();

private:
	bool equals(const uint8_t *buf1, const uint8_t *buf2, const size_t length) {
		for (size_t i = 0; i < length; i++) {
			if (buf1[i] != buf2[i]) {
				return false;
			}
		}
		return true;
	}

	bool equals(const cmd *cmd1, const cmd *cmd2) {
		return (cmd1->cmd_id == cmd2->cmd_id && cmd1->engine_id == cmd2->engine_id
			&& cmd1->direction == cmd2->direction && cmd1->data == cmd2->data);
	}

public:
	void testMarshalAndUnmarshal() {
		std::vector<struct test> tests {
			{{0, 0, 0, 0}, 		{0x0, 0x0}},		// stop left engine
			{{0, 1, 0, 0}, 		{0x2, 0x0}},		// stop right engine
			{{1, 0, 0, 127}, 	{0x10, 0x7f}},		// rotate left engine CW
			{{1, 0, 1, 10}, 	{0x11, 0xa}},		// rotate left engine CCW
			{{1, 1, 0, 255}, 	{0x12, 0xff}},		// rotate right engine CW
			{{1, 1, 1, 0},	 	{0x13, 0x0}},		// rotate right engine CCW
		};

		for (auto it = tests.begin(); it != tests.end(); it++) {
			auto test = (*it);
			
			// marshal
			uint8_t buf[2];
			marshal(&test.in, buf);
			CPPUNIT_ASSERT_EQUAL(true, equals(test.out, buf, 2));

			// unmarshal
			struct cmd newCmd;
			unmarshal(&newCmd, buf);
			CPPUNIT_ASSERT_EQUAL(true, equals(&test.in, &newCmd));
		}
	}

};
CPPUNIT_TEST_SUITE_REGISTRATION(EngineControlCommonTest);
CPPUNIT_TEST_SUITE_NAMED_REGISTRATION(EngineControlCommonTest, "EngineControlCommonTest");
