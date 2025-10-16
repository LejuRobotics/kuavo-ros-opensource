import resolve from '@rollup/plugin-node-resolve';
import commonjs from '@rollup/plugin-commonjs';
// import typescript from '@rollup/plugin-typescript';
import json from '@rollup/plugin-json';
import replace from '@rollup/plugin-replace';
import { readFileSync } from 'fs';

// 读取 package.json 获取版本信息
const pkg = JSON.parse(readFileSync('./package.json', 'utf8'));

// 外部依赖配置（不打包进bundle）
const external = [
  'ws',
  'eventemitter3',
  'path',
  'fs',
  'os',
  'events'
];

// 通用插件配置
const plugins = [
  replace({
    'process.env.NODE_ENV': JSON.stringify(process.env.NODE_ENV || 'production'),
    'process.env.SDK_VERSION': JSON.stringify(pkg.version),
    'process.env.PACKAGE_NAME': JSON.stringify(pkg.name),
    preventAssignment: true
  }),
  resolve({
    preferBuiltins: true,
    browser: false
  }),
  commonjs(),
  // typescript({
  //   tsconfig: './tsconfig.json',
  //   declaration: true,
  //   declarationDir: './dist',
  //   rootDir: './src'
  // }),
  json()
];

export default [
  // 主包构建
  {
    input: './index.js',
    output: [
      {
        file: 'dist/index.js',
        format: 'cjs',
        exports: 'named',
        sourcemap: true
      },
      {
        file: 'dist/index.esm.js', 
        format: 'es',
        sourcemap: true
      }
    ],
    plugins,
    external
  },
  
  // 消息包单独构建
  // {
  //   input: 'src/msg/index.ts',
  //   output: [
  //     {
  //       file: 'dist/msg/index.js',
  //       format: 'cjs',
  //       exports: 'named'
  //     },
  //     {
  //       file: 'dist/msg/index.esm.js',
  //       format: 'es'
  //     }
  //   ],
  //   plugins,
  //   external
  // },
  
  // 接口包单独构建
  // {
  //   input: 'src/interfaces/index.ts',
  //   output: [
  //     {
  //       file: 'dist/interfaces/index.js',
  //       format: 'cjs',
  //       exports: 'named'
  //     },
  //     {
  //       file: 'dist/interfaces/index.esm.js',
  //       format: 'es'
  //     }
  //   ],
  //   plugins,
  //   external
  // },
  
  // Kuavo核心包单独构建
  // {
  //   input: 'src/kuavo/index.ts',
  //   output: [
  //     {
  //       file: 'dist/kuavo/index.js',
  //       format: 'cjs',
  //       exports: 'named'
  //     },
  //     {
  //       file: 'dist/kuavo/index.esm.js',
  //       format: 'es'
  //     }
  //   ],
  //   plugins,
  //   external
  // },
  
  // 策略包单独构建
  // {
  //   input: 'src/kuavo_strategy/index.ts',
  //   output: [
  //     {
  //       file: 'dist/kuavo_strategy/index.js',
  //       format: 'cjs',
  //       exports: 'named'
  //     },
  //     {
  //       file: 'dist/kuavo_strategy/index.esm.js',
  //       format: 'es'
  //     }
  //   ],
  //   plugins,
  //   external
  // }
];