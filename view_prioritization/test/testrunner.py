"""View Prioritization Test Runner"""

import rosunit

if __name__ == "__main__":
    rosunit.unitrun(
        "view_prioritization",
        "test_prioritization",
        "test.test_prioritization.PrioritizationTestSuite",
    )
    rosunit.unitrun(
        "view_prioritization", "test_parser", "test.test_parser.ParserTestSuite"
    )
