#include <cppunit/extensions/HelperMacros.h>

class EngineControlCommonTest : public CppUnit::TestFixture {

	CPPUNIT_TEST_SUITE(EngineControlCommonTest);
	CPPUNIT_TEST(testMarshal);
	CPPUNIT_TEST_SUITE_END();

public:
	void testMarshal() {

		
		CPPUNIT_ASSERT(0 == 0);
	}

};
CPPUNIT_TEST_SUITE_REGISTRATION(EngineControlCommonTest);
CPPUNIT_TEST_SUITE_NAMED_REGISTRATION(EngineControlCommonTest, "EngineControlCommonTest");
