/* test_conditionSet.cpp
Copyright (c) 2020 by Benjamin Hauch

Endless Sky is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.

Endless Sky is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include "es-test.hpp"

// Include only the tested class's header.
#include "../../../source/ConditionSet.h"

// Include a helper for creating well-formed DataNodes (to enable creating non-empty ConditionSets).
#include "datanode-factory.h"
// Include a helper for capturing & asserting on logged output.
#include "output-capture.hpp"

// Include ConditionStore, to enable usage of them for testing ConditionSets.
#include "../../../source/ConditionsStore.h"

// ... and any system includes needed for the test file.
#include <cstdint>
#include <map>
#include <string>

namespace { // test namespace
using Conditions = std::map<std::string, int64_t>;
// #region mock data



// #endregion mock data



// #region unit tests
SCENARIO( "Creating a ConditionSet" , "[ConditionSet][Creation]" ) {
	GIVEN( "no arguments" ) {
		const auto set = ConditionSet{};
		THEN( "no conditions are created" ) {
			REQUIRE( set.IsEmpty() );
		}
	}
	GIVEN( "a node with no children" ) {
		auto childlessNode = AsDataNode("never");
		const auto set = ConditionSet{childlessNode};

		THEN( "no conditions are created" ) {
			REQUIRE( set.IsEmpty() );
		}
	}
	GIVEN( "a node with valid children" ) {
		auto nodeWithChildren = AsDataNode("and\n\tnever");
		const auto set = ConditionSet{nodeWithChildren};

		THEN( "a non-empty ConditionSet is created" ) {
			REQUIRE_FALSE( set.IsEmpty() );
		}
	}
}

SCENARIO( "Extending a ConditionSet", "[ConditionSet][Creation]" ) {

	OutputSink warnings(std::cerr);

	GIVEN( "an empty ConditionSet" ) {
		auto set = ConditionSet{};
		REQUIRE( set.IsEmpty() );

		THEN( "no expressions are added from empty nodes" ) {
			const std::string validationWarning = "Error: Loading empty (sub)condition:\ntoplevel\n\n";
			set.Load(AsDataNode("toplevel"));
			REQUIRE( set.IsEmpty() );
			AND_THEN( "a log message is printed to assist the user" ) {
				REQUIRE( warnings.Flush() == validationWarning );
			}
		}
		THEN( "no expressions are added from invalid nodes" ) {
			const std::string validationWarning = "Error: An expression must either perform a comparison or assign a value:\n";
			const std::string invalidNodeText = "and\n\thas";
			const std::string invalidNodeTextInWarning = "and\nL2:   has";
			set.Load(AsDataNode(invalidNodeText));
			REQUIRE( set.IsEmpty() );
			AND_THEN( "a log message is printed to assist the user" ) {
				REQUIRE( warnings.Flush() == validationWarning + invalidNodeTextInWarning + '\n' + '\n');
			}
		}
		THEN( "new expressions can be added from valid nodes" ) {
			set.Load(AsDataNode("and\n\tnever"));
			REQUIRE_FALSE( set.IsEmpty() );
			REQUIRE( warnings.Flush() == "" );
		}
	}
}

SCENARIO( "Determining if condition requirements are met", "[ConditionSet][Usage]" ) {
	GIVEN( "an empty ConditionSet" ) {
		const auto emptySet = ConditionSet{};
		REQUIRE( emptySet.IsEmpty() );

		AND_GIVEN( "an empty list of Conditions" ) {
			const auto emptyConditionList = ConditionsStore{};
			THEN( "the ConditionSet is satisfied" ) {
				REQUIRE( emptySet.Test(emptyConditionList) );
			}
		}
		AND_GIVEN( "a non-empty list of Conditions" ) {
			const auto conditionList = ConditionsStore {
				{"event: war begins", 1},
			};
			THEN( "the ConditionSet is satisfied" ) {
				REQUIRE( emptySet.Test(conditionList) );
			}
		}
	}
	GIVEN( "a set containing 'never'" ) {
		const auto neverSet = ConditionSet{AsDataNode("and\n\tnever")};
		REQUIRE_FALSE( neverSet.IsEmpty() );

		AND_GIVEN( "a condition list containing the literal 'never'" ) {
			const auto listWithNever = ConditionsStore {
				{"never", 1},
			};
			THEN( "the ConditionSet is not satisfied" ) {
				REQUIRE_FALSE( neverSet.Test(listWithNever) );
			}
		}
	}
}

// #endregion unit tests



} // test namespace
