use std::{collections::HashMap, path::PathBuf};

use serde::{Deserialize, Serialize};

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct CoreSettings {
    /// A list of files and their most recently selected paths.
    #[serde(rename = "_file_paths")]
    pub file_paths: Vec<(u16, PathBuf)>,

    #[serde(flatten)]
    pub settings: HashMap<String, serde_json::Value>,
}
