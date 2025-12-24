// plugins/env-plugin/index.js
// Custom Docusaurus plugin to configure environment variables for the frontend

module.exports = function (context, options) {
  return {
    name: 'env-plugin',

    configureWebpack(config, isServer, utils) {
      const { CreateReactAppUtils } = utils;

      return {
        define: {
          'process.env.REACT_APP_API_URL': JSON.stringify(process.env.REACT_APP_API_URL || process.env.RAG_API_URL || 'http://localhost:8000'),
          'process.env.RAG_API_URL': JSON.stringify(process.env.RAG_API_URL || process.env.REACT_APP_API_URL || 'http://localhost:8000'),
        },
      };
    },
  };
};