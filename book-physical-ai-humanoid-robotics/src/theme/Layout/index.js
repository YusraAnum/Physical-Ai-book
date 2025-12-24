import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import FloatingChatWidget from '@site/src/components/ChatInterface/FloatingChatWidget';

export default function LayoutWrapper(props) {
  return (
    <>
      <OriginalLayout {...props}>
        {props.children}
      </OriginalLayout>
      <FloatingChatWidget />
    </>
  );
}