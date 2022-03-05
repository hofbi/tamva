#ifndef VIEW_MDA_MDVQM_PARAM_H
#define VIEW_MDA_MDVQM_PARAM_H

#include <array>

namespace lmt::vqm
{
struct MDVQMParam
{
    static constexpr std::array<double, 6> a{-0.632, 0.6591, 4.1797, -0.0352, 0.1133, 5.6086};
    static constexpr double as{0.0222};
    static constexpr double at{0.0518};
    static constexpr double bs{0.0035};
    static constexpr double bt{0.7889};
};
}  // namespace lmt::vqm

#endif  // VIEW_MDA_MDVQM_PARAM_H
