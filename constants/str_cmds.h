#include <string>
#include <unordered_map>

enum AutonomousCmd { 
    RREVERSE 	, 
    LREVERSE	, 
    LCW 		, 
    RCW 		, 
    SPIN 		, 
    LCP 		, 
    RCP 		, 
    STOP 		, 
    PIVOTR 	, 
    PIVOTL 	, 
    FPIVOTR 	, 
    FPIVOTL 	, 
    VEERR 		, 
    VEERL 		, 
    FWD 		, 
    SLIGHTR 	, 
    SLIGHTL 	, 
    SPOOK 		, 
    GO 		,
};

static const std::unordered_map<std::string, AutonomousCmd> AUTOCMD_STRING_TO_ENUM = {
    {"RREVERSE", RREVERSE},
    {"LREVERSE" ,	LREVERSE},
    {"LCW" , LCW},
    {"RCW" , RCW},
    {"SPIN"  , SPIN},
    {"LCP" , LCP},
    {"RCP" , RCP},
    {"STOP" , STOP},
    {"PIVOTR" , PIVOTR},
    {"PIVOTL" , PIVOTL},
    {"FPIVOTR" , FPIVOTR},
    {"FPIVOTL" , FPIVOTL},
    {"VEERR" , VEERR},
    {"VEERL" , VEERL},
    {"FWD" , FWD},
    {"SLIGHTR" , SLIGHTR},
    {"SLIGHTL" , SLIGHTL},
    {"SPOOK" , SPOOK},
    {"GO" , GO},
<<<<<<< HEAD
};
=======
}
>>>>>>> a3e125733334221544ec8fe4e70d19bbb65bbeb6
