var ros = new ROSLIB.Ros({
  url: "ws://localhost:9090",
});

ros.getParams(function(params) {
  console.log(params);
});

var active_chair_nums = new ROSLIB.Param({
  ros: ros,
  name: "active_chair_nums",
});

var chairs = [];
var live_status = new Map();



// setActiveChairNums([2, 3, 4]);
// Get the modal
var modal = document.getElementById("myModal");

// Get the button that opens the modal
var btn = document.getElementById("editActiveChairsBtn");

// Get the <span> element that closes the modal
var span = document.getElementsByClassName("closeModal")[0];

// When the user clicks on the button, open the modal
btn.onclick = function() {
  modal.style.display = "block";
}

// When the user clicks on <span> (x), close the modal
span.onclick = function() {
  modal.style.display = "none";
}

// When the user clicks anywhere outside of the modal, close it
window.onclick = function(event) {
  if (event.target == modal) {
    modal.style.display = "none";
  }
}

function handleSetActiveChairs(e) {
    e.preventDefault();
    const formData = new FormData(e.target);
    const formProps = Object.fromEntries(formData);
    const newActiveChairs = Object.keys(formProps).map(id => parseInt(id));
    
    setActiveChairNums(newActiveChairs);
    modal.style.display = "none";

    var empty_msg = new ROSLIB.Message({});
    reload_active_chairs_pub.publish(empty_msg);
}

function setActiveChairNums(chairList) {
    active_chair_nums.set(chairList);
    console.log("GETTING ACTIVE CHAIRS!", chairList);
    chairs = chairList.map((v) => String(v));
    chairs.map((chair) => live_status.set(chair, null));

  chairs.forEach((chair) => document.getElementById("activate" + chair).checked = true);

  document.getElementById("num_chairs").innerHTML =
    String(chairs.length) + " chair" + (chairs.length != 1 ? "s" : "");
  document.getElementById("active_chair_list").innerHTML =
    chairs.toString();
    generateStatuses(chairList);

}

function generateStatuses(chairList) {
    console.log("generating statuses");
    const statusBlock = (id) => (`<div class="chair_monitor" id="${id}">
    <h2>Chair ${id}</h2>
    <div class="status o" id="${id}status">
      OFFLINE<span class="dot"></span>
    </div>
    <div class="status o" id="${id}_broadcast_status">OFFLINE</div>
    <div class="status o" id="${id}_stuck_status">OFFLINE</div>
    <div class="status o" id="${id}_trapped_status">OFFLINE</div>
    <div id="${id}_flags"></div>
    <br />
    <div class="chair_control">
      <div class="directions">
        <button class="auto" id="${id}" onclick="toggle(this.id)">
          AUTO
        </button>
        <br />
        <button id="${id}" onclick="ffwd(this.id)">⤊</button>
        <div>
          <button id="${id}" onclick="fwdl(this.id)">⬉</button>
          <button id="${id}" onclick="fwd(this.id)">↑</button>
          <button id="${id}" onclick="fwdr(this.id)">⬈</button>
        </div>
        <div>
          <button id="${id}" onclick="pivotl(this.id)">⟲</button>
          <button id="${id}" onclick="stop(this.id)">🛑</button>
          <button id="${id}" onclick="pivotr(this.id)">⟳</button>
        </div>
        <div>
          <button id="${id}" onclick="bwdl(this.id)">⬋</button>
          <button id="${id}" onclick="bwd(this.id)">↓</button>
          <button id="${id}" onclick="bwdr(this.id)">⬊</button>
        </div>
        <button id="${id}" onclick="fbwd(this.id)">⤋</button>
      </div>
      <br />
      <!-- <button id="${id}" onclick="start(this.id)">START</button>
  <button id="${id}" onclick="stop(this.id)" class="stop">STOP</button> -->
      <br />
      <form
        id="${id}custom_handwritten"
        class="handwritten_form"
        onsubmit="return submitForm(event);"
      >
        <label for="custom_handwritten">Custom: </label>
        <input
          type="text"
          id="custom_handwritten"
          name="custom_handwritten"
          size="10"
        />
        <button type="submit" class="submit_button">Send</button>
      </form>
    </div>
  </div>`);

  const statusContainer = document.getElementById("chairStatuses");
//   Clear out statuses
  statusContainer.innerHTML = '';

  chairList.forEach(id => statusContainer.insertAdjacentHTML( 'beforeend', statusBlock(id)));

}


var editor;


active_chair_nums.get(function (value) {
  setActiveChairNums(value);
});
// var chairs = ["1", "2", "3", "4"];
console.log(live_status);

var test_broadcast = [
  "00Bf2.0f2.0t5",
  "00BAp",
  "00Bf0.0f0.0t5",
  "00Bf0.0f0.0t5",
  "00BAk",
  "00Br2.0r2.0t5",
  "00Br2.0r2.0t5",
  "00BAt",
  "00Bf0.0f0.0t5",
  "00Bf0.0f0.0t5",
  "00Bend",
];

var cmdEquivalents = {
  fwd: {
    autonomous: [
      { key: "FWD", prefix: "DA" },
      { key: "GO", prefix: "FA" },
    ],
    handwritten: [{ key: "fwd", prefix: "0H" }],
  },
  ffwd: {
    autonomous: [{ key: "FFWD", prefix: "DA" }],
    handwritten: [{ key: "ffwd", prefix: "0H" }],
  },
  fwdl: {
    autonomous: [{ key: "VEERL", prefix: "DA" }],
    handwritten: [{ key: "fwdl", prefix: "0H" }],
  },
  fwdr: {
    autonomous: [{ key: "VEERR", prefix: "DA" }],
    handwritten: [{ key: "fwdr", prefix: "0H" }],
  },
  left: {
    autonomous: [{ key: "LEFT", prefix: "DA" }],
    handwritten: [{ key: "left", prefix: "0H" }],
  },
  right: {
    autonomous: [{ key: "RIGHT", prefix: "DA" }],
    handwritten: [{ key: "right", prefix: "0H" }],
  },
  bwd: {
    autonomous: [{ key: "BWD", prefix: "DA" }],
    handwritten: [{ key: "bwd", prefix: "0H" }],
  },
  fbwd: {
    autonomous: [{ key: "FBWD", prefix: "DA" }],
    handwritten: [{ key: "fbwd", prefix: "0H" }],
  },
  bwdl: {
    autonomous: [{ key: "BWDL", prefix: "DA" }],
    handwritten: [{ key: "bwdl", prefix: "0H" }],
  },
  bwdr: {
    autonomous: [{ key: "BWDR", prefix: "DA" }],
    handwritten: [{ key: "bwdr", prefix: "0H" }],
  },
  pivotl: {
    autonomous: [{ key: "PIVOTL", prefix: "DA" }],
    handwritten: [{ key: "pivotl", prefix: "0H" }],
  },
  pivotr: {
    autonomous: [{ key: "PIVOTR", prefix: "DA" }],
    handwritten: [{ key: "pivotr", prefix: "0H" }],
  },
  spook: {
    autonomous: [{ key: "SPOOK", prefix: "EA" }],
    handwritten: [],
  },
  // stop, toggle
  // fpivotr, fpivotl
};

const initialCmdsJson = {
  handwritten: {
    left: "0Hf0.0f3.0",
    right: "0Hf3.0f0.0",
    fwd: "0Hf2.0f2.0",
    ffwd: "0Hf3.0f3.0",
    fwdl: "0Hf1.5f3.0",
    fwdr: "0Hf3.0f1.5",
    bwd: "0Hr2.0r2.0",
    fbwd: "0Hr3.0r3.0",
    bwdl: "0Hr1.5r3.0",
    bwdr: "0Hr3.0r1.5",
    pivotl: "0Hr2.5f2.5",
    pivotr: "0Hf2.5r2.5",
    stop: "0Hf0.0f0.0",
    toggle: "0Htoggle",
  },
  autonomous: {
    RREVERSE: "ACD",
    LREVERSE: "ACE",
    LCW: "ACF",
    RCW: "ACG",
    SPIN: "ACH",
    LCP: "BCI",
    RCP: "BCJ",
    STOP: "CAf0.0f0.0",
    PIVOTR: "DAf2.5r2.5",
    PIVOTL: "DAr2.5f2.5",
    FPIVOTR: "DAf3.0r3.0",
    FPIVOTL: "DAr3.0f3.0",
    VEERR: "DAf3.0f1.5",
    VEERL: "DAf1.5f3.0",
    FWD: "DAf1.5f1.5",
    BWD: "DAr1.5r1.5",
    FFWD: "DAf2.0f2.0",
    FBWD: "DAr2.0r2.0",
    SLIGHTR: "DAf3.0f2.0",
    SLIGHTL: "DAf2.0f3.0",
    SPOOK: "EAf4.0f4.0",
    GO: "FAf1.5f1.5",
  },
};

ros.on("connection", function () {
  document.getElementById("status").innerHTML = "Connected";
});

ros.on("error", function (error) {
  document.getElementById("status").innerHTML = "Error";
});

ros.on("close", function () {
  document.getElementById("status").innerHTML = "Closed";
});

var txt_listener = new ROSLIB.Topic({
  ros: ros,
  name: "/from_hub_receiver",
  messageType: "std_msgs/String",
});

var hub_to_gui_listener = new ROSLIB.Topic({
  ros: ros,
  name: "/hub_to_gui",
  messageType: "std_msgs/String",
});

var to_hub_manager_pub = new ROSLIB.Topic({
  ros: ros,
  name: "/to_hub_manager",
  messageType: "std_msgs/String",
});

var reload_active_chairs_pub = new ROSLIB.Topic({
    ros: ros,
    name: "/reload_active_chairs",
    messageType: "std_msgs/Empty",
  });

getTextForStatus = function (status) {
  if (status == "Br") {
    return "READY";
  } else if (status == "h") {
    return "HEARTBEAT";
  } else if (status == "H") {
    return "HANDWRITTEN";
  } else if (status == "A") {
    return "AUTONOMOUS";
  } else if (status == "B") {
    return "BROADCAST";
  } else if (status == "C") {
    return "CHOREO";
  } else if (status == "Be") {
    return "EXCLUDE";
  } else if (status == "Bs") {
    return "SUCCESS";
  } else if (status == "Bf") {
    return "FAILURE";
  } else if (status == "Tt") {
    return "TRAPPED";
  } else if (status == "Tm") {
    return "NOT TRAPPED";
  } else if (status == "Ss") {
    return "STUCK";
  } else if (status == "Sn") {
    return "NOT STUCK";
  } else if (status == "o" || status == null) {
    return "OFFLINE";
  } else {
    return "ERROR: " + status;
  }
};

function updateChairStatus(element, status) {
  if (element) {
    element.innerHTML = getTextForStatus(status);
    element.classList.remove(...element.classList);
    element.classList.add("status", status);
  }
}

function updateHeartbeatStatus(key, status) {
  live_status.set(key, status);
  let element = document.getElementById(key + "status");
  updateChairStatus(element, status);
}

function updateFlags(
  chair_num,
  flag_A,
  flag_B,
  flag_C,
  flag_H,
  flag_T,
  flag_D,
  flag_S,
  flag_EOC,
  flag_SOB,
  flag_EOB
) {
  let element = document.getElementById(chair_num + "_flags");
  if (element) {
    element.innerHTML = `${flag_A == "y" ? " A " : " x "}
    ${flag_B == "y" ? " B " : " x "}
    ${flag_C == "y" ? " C " : " x "}
    ${flag_H == "y" ? " H " : " x "}
    ${flag_T == "y" ? " T " : " x "}
    ${flag_D == "y" ? " D " : " x "}
    ${flag_S == "y" ? " S " : " x "}
    ${flag_EOC == "y" ? "EOC" : " x "}
    ${flag_SOB == "y" ? "SOB" : " x "}
    ${flag_EOB == "y" ? "EOB" : " x "}`;
  }
}

function updateBroadcastStatus(key, status) {
  let element = document.getElementById(key + "_broadcast_status");
  if (element) {
    element.innerHTML = getTextForStatus(status);
    element.classList.remove(...element.classList);
    element.classList.add("status", status);
  }
}

function updateChairTrappedStatus(key, status) {
  let element = document.getElementById(key + "_trapped_status");
  if (element) {
    element.innerHTML = getTextForStatus(status);
    element.classList.remove(...element.classList);
    element.classList.add("status", status);
  }
}

function updateChairStuckStatus(key, status) {
  let element = document.getElementById(key + "_stuck_status");
  if (element) {
    element.innerHTML = getTextForStatus(status);
    element.classList.remove(...element.classList);
    element.classList.add("status", status);
  }
}

hub_to_gui_listener.subscribe(function (m) {
  // Update
  if (m.data.length == 16 && m.data[0] == "u") {
    const chair_num = m.data[1];
    const broadcast_status = m.data[2];
    const stuck_status = m.data[3];
    const trapped_status = m.data[4];
    const state_status = m.data[5];
    const flag_A = m.data[6];
    const flag_B = m.data[7];
    const flag_C = m.data[8];
    const flag_H = m.data[9];
    const flag_T = m.data[10];
    const flag_D = m.data[11];
    const flag_S = m.data[12];
    const flag_EOC = m.data[13];
    const flag_SOB = m.data[14];
    const flag_EOB = m.data[15];

    updateBroadcastStatus(chair_num, "B" + broadcast_status);
    updateChairStuckStatus(chair_num, "S" + stuck_status);
    updateChairTrappedStatus(chair_num, "T" + trapped_status);
    updateHeartbeatStatus(chair_num, state_status);
    updateFlags(
      chair_num,
      flag_A,
      flag_B,
      flag_C,
      flag_H,
      flag_T,
      flag_D,
      flag_S,
      flag_EOC,
      flag_SOB,
      flag_EOB
    );
  }
  // } else {
  //   playBeep();
  //   alert(m.data);
  // }
});

txt_listener.subscribe(function (m) {
  document.getElementById("msg").innerHTML = m.data;
  console.log(m);
  if (
    // broadcast
    m.data.length == 3 &&
    chairs.includes(m.data[0]) &&
    m.data[1] == "B"
  ) {
    updateBroadcastStatus(m.data[0], m.data[1] + m.data[2]);
  } else if (
    // heartbeat
    m.data.length == 2 &&
    chairs.includes(m.data[0]) &&
    (m.data[1] == "H" ||
      m.data[1] == "A" ||
      m.data[1] == "C" ||
      m.data[1] == "B")
  ) {
    // live_status.set(m.data[0], m.data[1]);
    updateHeartbeatStatus(m.data[0], m.data[1]);
  }
  // image
  if (m.data[1] == "i") {
    console.log(m.data);
  }
});

function checkStatus() {
  for (let [key, value] of live_status) {
    if (value == null) {
      updateHeartbeatStatus(key, "o");
    }
    live_status.set(key, null);
  }
  setTimeout(checkStatus, 1000);
}

checkStatus();

hub_dir_listener = new ROSLIB.Topic({
  ros: ros,
  name: "/from_hub",
  messageType: "std_msgs/String",
});

initJsonEditor = function () {
  const jsonSchema = {
    $schema: "https://json-schema.org/draft/2020-12/schema",
    $id: "https://json-schema.org/draft/2020-12/schema",
    title: "Pre-loaded Commands",
    description:
      "Handwritten / Autonomous commands and their associated speeds.",
    type: "object",
    properties: {
      handwritten: {
        description: "Handwritten commands",
        type: "object",
        properties: {
          left: {
            type: "string",
          },
          right: {
            type: "string",
          },
          fwd: {
            type: "string",
          },
          ffwd: {
            type: "string",
          },
          fwdl: {
            type: "string",
          },
          fwdr: {
            type: "string",
          },
          bwd: {
            type: "string",
          },
          fbwd: {
            type: "string",
          },
          bwdl: {
            type: "string",
          },
          bwdr: {
            type: "string",
          },
          pivotl: {
            type: "string",
          },
          pivotr: {
            type: "string",
          },
          stop: {
            type: "string",
          },
          toggle: {
            type: "string",
          },
        },
        required: [
          "left",
          "right",
          "fwd",
          "ffwd",
          "fwdl",
          "fwdr",
          "bwd",
          "fbwd",
          "bwdl",
          "bwdr",
          "pivotl",
          "pivotr",
          "stop",
          "toggle",
        ],
      },
      autonomous: {
        description: "Autonomous commands",
        type: "object",
        properties: {
          RREVERSE: {
            type: "string",
          },
          LREVERSE: {
            type: "string",
          },
          LCW: {
            type: "string",
          },
          RCW: {
            type: "string",
          },
          SPIN: {
            type: "string",
          },
          LCP: {
            type: "string",
          },
          RCP: {
            type: "string",
          },
          STOP: {
            type: "string",
          },
          PIVOTR: {
            type: "string",
          },
          PIVOTL: {
            type: "string",
          },
          FPIVOTR: {
            type: "string",
          },
          FPIVOTL: {
            type: "string",
          },
          VEERR: {
            type: "string",
          },
          VEERL: {
            type: "string",
          },
          FWD: {
            type: "string",
          },
          BWD: {
            type: "string",
          },
          FFWD: {
            type: "string",
          },
          FBWD: {
            type: "string",
          },
          SLIGHTR: {
            type: "string",
          },
          SLIGHTL: {
            type: "string",
          },
          SPOOK: {
            type: "string",
          },
          GO: {
            type: "string",
          },
        },
        required: [
          "RREVERSE",
          "LREVERSE",
          "LCW",
          "RCW",
          "SPIN",
          "LCP",
          "RCP",
          "STOP",
          "PIVOTR",
          "PIVOTL",
          "FPIVOTR",
          "FPIVOTL",
          "VEERR",
          "VEERL",
          "FWD",
          "BWD",
          "FFWD",
          "FBWD",
          "SLIGHTR",
          "SLIGHTL",
          "SPOOK",
          "GO",
        ],
      },
    },
  };

  const container = document.getElementById("jsoneditor");
  const options = {
    schema: jsonSchema,
    mode: "form",
    onValidate: function (json) {
      // rules:
      // - team, names, and ages must be filled in and be of correct type
      // - a team must have 4 members
      // - at lease one member of the team must be adult
      const errors = [];

      if (json && !json.handwritten) {
        errors.push({
          path: ["handwritten"],
          message: "Handwritten commands must be specified",
        });
      }
      if (json && !json.autonomous) {
        errors.push({
          path: ["autonomous"],
          message: "Autonomous commands must be specified",
        });
      }
      if (json && json.handwritten) {
        // check whether each team member has name and age filled in correctly
        json.handwritten.properties.forEach(function (cmd, index) {
          if (typeof cmd !== "string") {
            errors.push({
              path: ["cmd", index],
              message: "Command must be a string",
            });
          }
          if (cmd[0] != "0" || cmd[1] != "H") {
            errors.push({
              path: ["cmd", index],
              message: 'Handwritten command must begin with "0H"',
            });
          }
        });
      }

      return errors;
    },
  };
  editor = new JSONEditor(container, options);
};

formPropsToString = function (formProps) {
  left_f_r = formProps.left_fwd_back == "left_f" ? "f" : "r";
  right_f_r = formProps.right_fwd_back == "right_f" ? "f" : "r";
  left_speed = Number.parseFloat(formProps.left_speed)
    .toFixed(1)
    .toString();
  right_speed = Number.parseFloat(formProps.right_speed)
    .toFixed(1)
    .toString();

  return left_f_r + left_speed + right_f_r + right_speed;
};

saveCmd = function (e) {
  e.preventDefault();
  console.log("prevented default");
  const formData = new FormData(e.target);
  const formProps = Object.fromEntries(formData);

  const stringcmd = formPropsToString(formProps);

  const matchedCmds = cmdEquivalents[formProps.cmd];
  console.log(matchedCmds);

  var json = editor.get();
  console.log(json);

  matchedCmds.handwritten.forEach((h) => {
    json.handwritten[h.key] = h.prefix + stringcmd;
  });
  matchedCmds.autonomous.forEach((a) => {
    json.autonomous[a.key] = a.prefix + stringcmd;
  });

  editor.set(json);
  editor.expandAll();
};

submitForm = function (e) {
  e.preventDefault();
  const formData = new FormData(e.target);
  const formProps = Object.fromEntries(formData);

  const cmd = formProps.custom_handwritten;

  if (
    cmd.length == 8 &&
    (cmd[0] == "f" || cmd[0] == "r") &&
    cmd[1] >= "0" &&
    cmd[1] <= "9" &&
    cmd[2] == "." &&
    cmd[3] >= "0" &&
    cmd[3] <= "9" &&
    (cmd[4] == "f" || cmd[4] == "r") &&
    cmd[5] >= "0" &&
    cmd[5] <= "9" &&
    (cmd[6] = ".") &&
    cmd[7] >= "0" &&
    cmd[7] <= "9"
  ) {
    hub_dir_listener.publish(
      formatMsg(e.target.id[0], "hand " + formProps.custom_handwritten)
    );
    document.getElementById(e.target.id).reset();
  } else {
    alert(
      "Custom commands must be 8 characters: [f/r][#.#][f/r][#.#]\nExample: f1.0f1.0"
    );
  }
};

move = function () {
  var dir = new ROSLIB.Message({
    data: "hello",
  });
  hub_dir_listener.publish(dir);
};

formatMsg = function (id, msg) {
  return new ROSLIB.Message({
    data: id + msg,
  });
};

toggle = function (id) {
  hub_dir_listener.publish(formatMsg(id, "toggle"));
};
stop = function (id) {
  hub_dir_listener.publish(formatMsg(id, "stop"));
};
start = function (id) {
  hub_dir_listener.publish(formatMsg(id, "start"));
};
fwd = function (id) {
  hub_dir_listener.publish(formatMsg(id, "fwd"));
};
ffwd = function (id) {
  hub_dir_listener.publish(formatMsg(id, "ffwd"));
};
bwd = function (id) {
  hub_dir_listener.publish(formatMsg(id, "bwd"));
};
fbwd = function (id) {
  hub_dir_listener.publish(formatMsg(id, "fbwd"));
};
fwdl = function (id) {
  hub_dir_listener.publish(formatMsg(id, "fwdl"));
};
fwdr = function (id) {
  hub_dir_listener.publish(formatMsg(id, "fwdr"));
};
bwdl = function (id) {
  hub_dir_listener.publish(formatMsg(id, "bwdl"));
};
bwdr = function (id) {
  hub_dir_listener.publish(formatMsg(id, "bwdr"));
};
pivotl = function (id) {
  hub_dir_listener.publish(formatMsg(id, "pivotl"));
};
pivotr = function (id) {
  hub_dir_listener.publish(formatMsg(id, "pivotr"));
};

send_test_broadcast = function () {
  console.log("SENDING TEST CHOREO");
  test_broadcast.forEach((msg) => {
    var dir = new ROSLIB.Message({
      data: msg,
    });
    to_hub_manager_pub.publish(dir);
  });
};

send_clear_broadcast = function () {
  console.log("SENDING CLEAR CHOREO");
  msg = "clear";
  var dir = new ROSLIB.Message({
    data: msg,
  });
  to_hub_manager_pub.publish(dir);
};

function switchscreen(newScreenId) {
  var liveScreen = document.getElementById("live");
  var configScreen = document.getElementById("config");
  if (newScreenId == "live") {
    configScreen.style.display = "none";
    liveScreen.style.display = "block";
  } else {
    liveScreen.style.display = "none";
    configScreen.style.display = "block";
  }
}

function renderJSON(obj) {
  "use strict";
  var keys = [],
    retValue = "";
  for (var key in obj) {
    if (typeof obj[key] === "object") {
      retValue += "<div class='tree'>" + key;
      retValue += renderJSON(obj[key]);
      retValue += "</div>";
    } else {
      retValue +=
        "<div class='tree'>" + key + " = " + obj[key] + "</div>";
    }

    keys.push(key);
  }
  console.log(retValue);
  return retValue;
}

openYaml = function (e) {
  console.log("OPENING YAML");
  e.preventDefault();
  console.log(e.target);
  const formData = new FormData(e.target);
  const formProps = Object.fromEntries(formData);

  const file = formProps.configFile;
  let ymlcontents = null;

  const reader = new FileReader();
  reader.addEventListener("load", (event) => {
    // event.target.result;
    ymlcontents = event.target.result.replace(/\r/g, "\n");
    // alert(ymlcontents);
    console.log(ymlcontents);

    document.getElementById("ymlTextArea").innerHTML = ymlcontents;

    const doc = jsyaml.load(ymlcontents);
    console.log(doc);

    // document.getElementById('ymlContainer').insertAdjacentHTML('beforeend', renderJSON(doc));
  });
  reader.readAsText(file);
};

saveYaml = function (e) {
  console.log("SAVING YAML");
  e.preventDefault();

  const textToSave = document.getElementById("ymlTextArea").innerHTML;
  const filename = document
    .getElementById("configFile")
    .files.item(0).name;

  var blob = new Blob([textToSave], { type: "text/plain;charset=utf-8" });
  saveAs(blob, filename);
};

sendYaml = function () {
  const filename = document
    .getElementById("configFile")
    .files.item(0).name;
  const textToSend =
    "config " +
    filename +
    " " +
    document.getElementById("ymlTextArea").innerHTML;
  hub_dir_listener.publish(formatMsg("0", textToSend));
};

sendCameraConfig = function (e) {
  e.preventDefault();

  const config = {
    camera: {
      topBandWidth: topWidth,
      sideBandWidth: sideWidth,
      imageWidth: imageWidth,
      imageHeight: imageHeight,
      brightnessThreshold: brightnessThreshold,
      sidePercentThreshold: sidePercent,
      topPercentThreshold: topPercent,
      numMiddlePixels:
        imageWidth * imageHeight -
        topWidth * imageWidth -
        2 * sideWidth * imageHeight +
        2 * topWidth * sideWidth,
    },
  };
  const textToSend = "config camera.yaml " + JSON.stringify(config);
  console.log(textToSend);
  hub_dir_listener.publish(formatMsg("0", textToSend));
};

const getColorIndicesForCoord = (x, y, width) => {
  const red = y * (width * 4) + x * 4;
  return [red, red + 1, red + 2, red + 3];
};

function setPixel(imageData, x, y, r, g, b, a) {
  index = (x + y * imageData.width) * 4;
  imageData.data[index + 0] = r;
  imageData.data[index + 1] = g;
  imageData.data[index + 2] = b;
  imageData.data[index + 3] = a;
}

var brightnessThreshold = 0;
var sideWidth = 0;
var topWidth = 0;
var sidePercent = 0;
var topPercent = 0;
const imageWidth = 640;
const imageHeight = 480;

function setBrightnessThreshold(newValue) {
  brightnessLabel.value = newValue;
  brightnessThreshold = newValue;
  drawImageBrightness();
}

function setSideWidth(newValue) {
  sideWidthLabel.value = newValue;
  sideWidth = newValue;
  drawImageBrightness();
}

function setTopWidth(newValue) {
  topWidthLabel.value = newValue;
  topWidth = newValue;
  drawImageBrightness();
}

function setSidePercent(newValue) {
  sidePercentLabel.value = newValue;
  sidePercent = newValue;
  drawImageBrightness();
}

function setTopPercent(newValue) {
  topPercentLabel.value = newValue;
  topPercent = newValue;
  drawImageBrightness();
}

function drawImageScaled(img, ctx, brightnessThreshold) {
  var canvas = ctx.canvas;
  var hRatio = canvas.width / img.width;
  var vRatio = canvas.height / img.height;
  var ratio = Math.min(hRatio, vRatio);
  var centerShift_x = (canvas.width - img.width * ratio) / 2;
  var centerShift_y = (canvas.height - img.height * ratio) / 2;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.drawImage(
    img,
    0,
    0,
    img.width,
    img.height,
    centerShift_x,
    centerShift_y,
    img.width * ratio,
    img.height * ratio
  );
}

function inTop(counter) {
  return counter < imageWidth * topWidth;
}

function inLeft(counter) {
  return counter % imageWidth < sideWidth;
}

function inRight(counter) {
  return counter % imageWidth > imageWidth - sideWidth;
}

function inMiddle(counter) {
  return !(
    inTop(counter, imageWidth, topWidth) ||
    inLeft(counter, imageWidth, sideWidth) ||
    inRight(counter, imageWidth, sideWidth)
  );
}

function drawImageBrightness() {
  var topCount = 0;
  var leftCount = 0;
  var rightCount = 0;
  var middleCount = 0;

  console.log(sidePercent, topPercent);

  const ctx = document.getElementById("canvas").getContext("2d");
  const imgctx = document.getElementById("bottomCanvas").getContext("2d");

  var camImage = imgctx.getImageData(0, 0, canvas.width, canvas.height);
  var data = camImage.data;
  newImage = ctx.createImageData(canvas.width, canvas.height);

  for (var i = 0; i < data.length; i += 4) {
    var brightness = (data[i] + data[i + 1] + data[i + 2]) / 3;
    const count = Math.floor(i / 4);
    if (inTop(count)) {
      topCount += brightness >= brightnessThreshold;
    }
    if (inLeft(count)) {
      leftCount += brightness >= brightnessThreshold;
    }
    if (inRight(count)) {
      rightCount += brightness >= brightnessThreshold;
    }
    if (inMiddle(count)) {
      middleCount += brightness >= brightnessThreshold;
    }
    if (brightness >= brightnessThreshold) {
      newImage.data[i] = 255;
      newImage.data[i + 1] = 0;
      newImage.data[i + 2] = 255;
      newImage.data[i + 3] = 255;
    } else {
      newImage.data[i] = data[i];
      newImage.data[i + 1] = data[i + 1];
      newImage.data[i + 2] = data[i + 2];
      newImage.data[i + 3] = 0;
    }
    if (i % 4 > 307000) {
      console.log(i);
    }
  }
  ctx.putImageData(newImage, 0, 0);

  // draw a red line
  ctx.strokeStyle = "lime";
  ctx.lineWidth = 3;

  const leftAverage = leftCount / (canvas.height * sideWidth);
  const rightAverage = rightCount / (canvas.height * sideWidth);
  const topAverage = topCount / (canvas.width * topWidth);

  console.log(leftAverage, rightAverage, topAverage);

  // left
  ctx.strokeStyle = leftAverage > sidePercent ? "red" : "lime";
  ctx.beginPath();
  ctx.moveTo(sideWidth, 0);
  ctx.lineTo(sideWidth, canvas.height);
  ctx.stroke();

  // right
  ctx.strokeStyle = rightAverage > sidePercent ? "red" : "lime";
  ctx.beginPath();
  ctx.moveTo(canvas.width - sideWidth, 0);
  ctx.lineTo(canvas.width - sideWidth, canvas.height);
  ctx.stroke();

  // top
  ctx.strokeStyle = topAverage > topPercent ? "red" : "lime";
  ctx.beginPath();
  ctx.moveTo(0, topWidth);
  ctx.lineTo(canvas.width, topWidth);
  ctx.stroke();
}

function toDataURL(url, callback) {
  var xhr = new XMLHttpRequest();
  xhr.onload = function () {
    var reader = new FileReader();
    reader.onloadend = function () {
      callback(reader.result);
    };
    reader.readAsDataURL(xhr.response);
  };
  xhr.open("GET", url);
  xhr.responseType = "blob";
  xhr.send();
}

function playLowBatt() {
  const audio = document.getElementById("audio_lowbatt");
  audio.play();
}

function playBeep() {
  const audio = document.getElementById("audio_beep");
  audio.play();
}

window.onload = function () {
  switchscreen("live");

  const canvas = document.getElementById("canvas");
  canvas.width = 640;
  canvas.height = 480;

  console.log("width:", canvas.width);
  console.log("height:", canvas.height);

  const ctx = canvas.getContext("2d", { willReadFrequently: true });

  const brightnessLabel = document.getElementById("brightnessLabel");
  brightnessThreshold = parseFloat(brightnessLabel.value);
  const sideWidthLabel = document.getElementById("sideWidthLabel");
  sideWidth = parseFloat(sideWidthLabel.value);
  const topWidthLabel = document.getElementById("topWidthLabel");
  topWidth = parseFloat(topWidthLabel.value);
  const sidePercentLabel = document.getElementById("sidePercentLabel");
  sidePercent = parseFloat(sidePercentLabel.value);
  const topPercentLabel = document.getElementById("topPercentLabel");
  topPercent = parseFloat(topPercentLabel.value);
  console.log(sidePercent, topPercent);

  const bottomCanvas = document.getElementById("bottomCanvas");
  bottomCanvas.width = 640;
  bottomCanvas.height = 480;
  const imgctx = bottomCanvas.getContext("2d", {
    willReadFrequently: true,
  });

  var background = new Image();
  background.crossOrigin = "Anonymous";
  background.src = "./images/test.jpg";

  toDataURL("./images/test.jpg", function (dataUrl) {
    console.log("RESULT:", dataUrl);
  });

  background.onload = function (event) {
    drawImageScaled(background, imgctx, ctx, brightnessThreshold);
    setBrightnessThreshold(brightnessThreshold);
    setSideWidth(sideWidth);
    setTopWidth(topWidth);
  };

  // set json
  initJsonEditor();
  editor.set(initialCmdsJson);
  editor.expandAll();

  // get json
  const updatedJson = editor.get();
  console.log(updatedJson);

  var list = document.getElementById("cmds");

  console.log(Object.keys(cmdEquivalents));
  Object.keys(cmdEquivalents).forEach(function (item) {
    var option = document.createElement("option");
    option.value = item;
    list.appendChild(option);
  });

  // playLowBatt();
  playBeep();
};