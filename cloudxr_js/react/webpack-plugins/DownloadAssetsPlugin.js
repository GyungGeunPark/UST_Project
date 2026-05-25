/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

const fs = require('fs');
const path = require('path');
const https = require('https');

class DownloadAssetsPlugin {
  constructor(assets) {
    this.assets = assets;
    this.hasRun = false;
    this.createdDirs = new Set();
  }

  safeUnlink(filePath) {
    try {
      fs.unlinkSync(filePath);
    } catch (err) {
      // Ignore cleanup errors
    }
  }

  apply(compiler) {
    compiler.hooks.beforeCompile.tapAsync('DownloadAssetsPlugin', (params, callback) => {
      // Only run once per webpack process
      if (this.hasRun) {
        callback();
        return;
      }

      console.log('📦 Checking and downloading required assets...');

      const downloadPromises = this.assets.map(asset => this.downloadFile(asset));

      Promise.allSettled(downloadPromises).then(results => {
        const failed = results.filter(r => r.status === 'rejected');
        if (failed.length > 0) {
          console.warn(`⚠️  ${failed.length} asset(s) failed to download, continuing anyway...`);
        }
        console.log('✅ Asset check complete!');
        this.hasRun = true;
        callback();
      });
    });
  }

  downloadFile({ url, output }) {
    return new Promise((resolve, reject) => {
      // Ensure directory exists (only once per unique path)
      if (!this.createdDirs.has(output)) {
        fs.mkdirSync(output, { recursive: true });
        this.createdDirs.add(output);
      }

      const filename = path.basename(url);
      const filePath = path.join(output, filename);

      // Skip if file already exists
      if (fs.existsSync(filePath)) {
        resolve();
        return;
      }

      console.log(`  Downloading ${filename}...`);

      const file = fs.createWriteStream(filePath);

      const downloadFromUrl = downloadUrl => {
        https
          .get(downloadUrl, response => {
            // Handle redirects
            if (response.statusCode === 302 || response.statusCode === 301) {
              file.close();
              this.safeUnlink(filePath);
              downloadFromUrl(response.headers.location);
              return;
            }

            if (response.statusCode !== 200) {
              file.close();
              this.safeUnlink(filePath);
              reject(new Error(`Failed to download ${filename}: HTTP ${response.statusCode}`));
              return;
            }

            response.pipe(file);

            file.on('finish', () => {
              file.close();
              console.log(`  ✓ Downloaded ${filename}`);
              resolve();
            });

            file.on('error', err => {
              this.safeUnlink(filePath);
              reject(err);
            });
          })
          .on('error', err => {
            if (fs.existsSync(filePath)) {
              this.safeUnlink(filePath);
            }
            reject(err);
          });
      };

      downloadFromUrl(url);
    });
  }
}

module.exports = DownloadAssetsPlugin;
