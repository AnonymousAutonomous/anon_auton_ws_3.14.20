#include <string>
#include <unordered_map>

enum AutonomousCmd
{
    RREVERSE,
    LREVERSE,
    LCW,
    RCW,
    SPIN,
    LCP,
    RCP,
    STOP,
    PIVOTR,
    PIVOTL,
    FPIVOTR,
    FPIVOTL,
    VEERR,
    VEERL,
    FWD,
    BWD,
    FFWD,
    FBWD,
    SLIGHTR,
    SLIGHTL,
    SPOOK,
    GO,
};

static std::unordered_map<std::string, AutonomousCmd> AUTOCMD_STRING_TO_ENUM = {
    {"RREVERSE", RREVERSE},
    {"LREVERSE", LREVERSE},
    {"LCW", LCW},
    {"RCW", RCW},
    {"SPIN", SPIN},
    {"LCP", LCP},
    {"RCP", RCP},
    {"STOP", STOP},
    {"PIVOTR", PIVOTR},
    {"PIVOTL", PIVOTL},
    {"FPIVOTR", FPIVOTR},
    {"FPIVOTL", FPIVOTL},
    {"VEERR", VEERR},
    {"VEERL", VEERL},
    {"FWD", FWD},
    {"BWD", BWD},
    {"FFWD", FFWD},
    {"FBWD", FBWD},
    {"SLIGHTR", SLIGHTR},
    {"SLIGHTL", SLIGHTL},
    {"SPOOK", SPOOK},
    {"GO", GO},
};

enum HandwrittenCmd
{
    H_LEFT,
    H_RIGHT,
    H_FWD,
    H_FFWD,
    H_FWDL,
    H_FWDR,
    H_BWD,
    H_FBWD,
    H_BWDL,
    H_BWDR,
    H_PIVOTL,
    H_PIVOTR,
    H_STOP,
    H_TOGGLE,
};

static std::unordered_map<std::string, HandwrittenCmd> HANDWRITTEN_CMD_TO_ENUM = {
    {"left", H_LEFT},
    {"right", H_RIGHT},
    {"fwd", H_FWD},
    {"ffwd", H_FFWD},
    {"fwdl", H_FWDL},
    {"fwdr", H_FWDR},
    {"bwd", H_BWD},
    {"fbwd", H_FBWD},
    {"bwdl", H_BWDL},
    {"bwdr", H_BWDR},
    {"pivotl", H_PIVOTL},
    {"pivotr", H_PIVOTR},
    {"stop", H_STOP},
    {"toggle", H_TOGGLE},
};

enum ChoreoCmd
{
    DANCE_C,
    RREVERSE_C,
    LREVERSE_C,
    LCP_C,
    RCP_C,
    SPIN_C,
    AUDIO_C,
};

static std::unordered_map<std::string, ChoreoCmd> CHOREO_CMD_TO_ENUM = {
    {"DANCE_C", DANCE_C},
    {"RREVERSE_C", RREVERSE_C},
    {"LREVERSE_C", LREVERSE_C},
    {"LCP_C", LCP_C},
    {"RCP_C", RCP_C},
    {"SPIN_C", SPIN_C},
    {"AUDIO_C", AUDIO_C},
};
